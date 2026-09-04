"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The rules the late join has to keep.

No hardware and no link: the point of the joining state is that it is pure
plumbing around two model states, so both of those are fakes here. What is
actually being pinned is the four things that are easy to get wrong and
expensive to find on the car - that modeld gets a working model immediately,
that a swap never lands on an engaged frame, that a large model which dies
mid-drive falls back without losing the frame, and that a small-model failure
still belongs to modeld.
"""
import re
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from openpilot.common.basedir import BASEDIR

from openpilot.sunnypilot.accelerators.jetlink.joining import STABLE_SECONDS, JoiningModelState


class FakeModel:
  def __init__(self, name, chestnut=False, client=None):
    self.name = name
    self.chestnut = chestnut
    self.client = client
    self.lat_delay = 0.0
    self.vision_input_names = ['img', 'big_img']
    self.calls = 0
    self.raises = None
    self.closed = False
    self.warmed = False

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    self.calls += 1
    if self.raises is not None:
      raise self.raises
    return {'from': self.name}

  def warmup(self):
    self.warmed = True

  def close(self):
    self.closed = True


class JoiningTest(unittest.TestCase):
  def setUp(self):
    # The engagement watcher is the one part that needs msgq. Drive the flag by
    # hand instead; what it reads is covered by selfdrived's own tests.
    patcher = mock.patch.object(JoiningModelState, '_watch_engagement', lambda self: None)
    patcher.start()
    self.addCleanup(patcher.stop)

    self.params = {}
    fake_params = mock.MagicMock()
    fake_params.put_bool.side_effect = lambda k, v: self.params.__setitem__(k, v)
    fake_params.remove.side_effect = lambda k: self.params.pop(k, None)
    patcher = mock.patch('openpilot.sunnypilot.accelerators.jetlink.joining.Params',
                         return_value=fake_params)
    patcher.start()
    self.addCleanup(patcher.stop)

    self.small = FakeModel('small')
    self.big = FakeModel('big', chestnut=True, client=object())
    self.joined = threading.Event()
    self.connect_calls = 0
    self.connect_error = None

  def _connect(self):
    self.connect_calls += 1
    if self.connect_error is not None:
      raise self.connect_error
    self.joined.set()
    # A link the joining state may have to close on its own, when it is torn
    # down holding a join that never found a disengaged frame to land on.
    return (mock.MagicMock(name='client'), 'spec')

  def _build(self, client, spec):
    return self.big

  def _state(self):
    s = JoiningModelState(1928, 1208, self.small, self._connect, self._build)
    self.addCleanup(s.close)
    return s

  def _run(self, s):
    return s.run({}, {}, {})

  def test_runs_the_small_model_immediately(self):
    s = self._state()
    # No waiting on a link: this is the whole point.
    self.assertEqual(self._run(s), {'from': 'small'})
    self.assertFalse(s.chestnut)
    self.assertIsNone(s.client)

  def test_does_not_swap_while_engaged_and_moving(self):
    s = self._state()
    self.assertTrue(self.joined.wait(5.0))
    s._engaged, s._standstill = True, False
    for _ in range(3):
      self.assertEqual(self._run(s), {'from': 'small'})
    self.assertFalse(s.chestnut)
    self.assertFalse(self.big.warmed)

  def test_swaps_at_a_standstill_while_engaged(self):
    # The drive that gave nothing else: engaged on the ramp, off at the exit.
    # Stopped, the plan is steering nothing, so the step lands on nothing and
    # the Jetson gets used instead of sitting ready for an hour.
    s = self._state()
    self.assertTrue(self.joined.wait(5.0))
    s._engaged, s._standstill = True, True
    self.assertEqual(self._run(s), {'from': 'big'})
    self.assertTrue(s.chestnut)

  def test_a_standstill_the_car_stopped_reporting_is_not_a_window(self):
    # sm.alive goes false when carState stops arriving. A stale "stopped" must
    # not open a window on a car that is actually moving.
    s = self._state()
    self.assertTrue(self.joined.wait(5.0))
    s._engaged, s._standstill = True, False
    self.assertEqual(self._run(s), {'from': 'small'})

  def test_swaps_on_a_disengaged_frame(self):
    s = self._state()
    self.assertTrue(self.joined.wait(5.0))
    s._engaged = False
    self.assertEqual(self._run(s), {'from': 'big'})
    self.assertTrue(s.chestnut)
    self.assertIs(s.client, self.big.client)
    # No warmup at the swap: the warp was prepared in __init__ and a frame
    # over the link here was two dropped camera frames on the car.
    self.assertFalse(self.big.warmed)

  def test_large_model_failure_demotes_and_keeps_the_frame(self):
    s = self._state()
    self.assertTrue(self.joined.wait(5.0))
    s._engaged = False
    self._run(s)
    self.assertTrue(s.chestnut)

    self.big.raises = RuntimeError("link gone")
    # modeld still gets an output for this frame, from the small model.
    self.assertEqual(self._run(s), {'from': 'small'})
    self.assertFalse(s.chestnut)
    self.assertTrue(self.big.closed)
    # And it tries again rather than staying small for the rest of the drive.
    self.assertTrue(s._rejoin.is_set() or self.connect_calls > 1)

  def test_prepare_runs_in_the_constructor_and_its_failure_is_survived(self):
    order = []
    self.connect_calls = 0

    def prepare():
      order.append('prepare')
      raise RuntimeError("no warp today")

    s = JoiningModelState(1928, 1208, self.small, self._connect, self._build, prepare)
    self.addCleanup(s.close)
    # Ran, and ran before anything else: modeld's main thread is blocked for
    # exactly as long as the constructor takes, so this is the only place the
    # GPU work can go without costing a frame.
    self.assertEqual(order, ['prepare'])
    self.assertEqual(self._run(s), {'from': 'small'})
    # Still joins; the swap will just pay for the warp itself.
    self.assertTrue(self.joined.wait(5.0))
    s._engaged = False
    self.assertEqual(self._run(s), {'from': 'big'})

  def test_loading_has_no_deadline(self):
    # It used to end at 60 s because it was a NO_ENTRY in selfdrived. It is not
    # any more (selfdrived only gates on it while nothing publishes modelV2),
    # so a Jetson that takes a whole drive to arrive stays "getting ready" and
    # the join keeps going. The 60 s edge used to read as "Big Model Ready" to
    # selfdrived and "unavailable" to the UI, both false.
    self.connect_error = RuntimeError("jetson still booting")
    s = self._state()
    self._run(s)
    with mock.patch('openpilot.sunnypilot.accelerators.jetlink.joining.time.monotonic',
                    return_value=time.monotonic() + 600.0):
      self._run(s)
    self.assertIs(self.params.get("ChestnutLoading"), True)
    self.assertTrue(s.loading)
    self.assertNotIn("ChestnutActive", self.params)
    # A join that succeeds later still swaps.
    self.connect_error = None
    s._rejoin.set()
    self.assertTrue(self.joined.wait(10.0))
    s._engaged = False
    for _ in range(20):
      if self._run(s) == {'from': 'big'}:
        break
      time.sleep(0.05)
    self.assertTrue(s.chestnut)
    self.assertFalse(s.loading)

  def test_owns_the_params_modeld_would_write(self):
    # From the constructor, before modeld's main thread gets the object back:
    # loading is true and active is neither true nor false. modeld reads
    # `loading` and leaves both alone. True/false at the swap, false/true at a
    # demote, so selfdrived's "Big Model Ready" edge is the swap and its "Big
    # Model Failed" soft disable is the demote, as they are for a chestnut.
    self.params['ChestnutActive'] = True   # stale, from whatever ran before
    s = self._state()
    self.assertIs(self.params.get('ChestnutLoading'), True)
    self.assertNotIn('ChestnutActive', self.params)
    self.assertTrue(s.loading)

    self.assertTrue(self.joined.wait(5.0))
    s._engaged = False
    self._run(s)
    self.assertIs(self.params.get('ChestnutActive'), True)
    self.assertIs(self.params.get('ChestnutLoading'), False)
    self.assertFalse(s.loading)

    self.big.raises = RuntimeError("link gone")
    self._run(s)
    self.assertIs(self.params.get('ChestnutActive'), False)
    self.assertIs(self.params.get('ChestnutLoading'), True)
    self.assertTrue(s.loading)

  def test_build_failure_backs_off(self):
    self._build = mock.Mock(side_effect=RuntimeError("no warp"))
    s = JoiningModelState(1928, 1208, self.small, self._connect, self._build)
    self.addCleanup(s.close)
    self.assertTrue(self.joined.wait(5.0))
    s._engaged = False
    self.assertEqual(self._run(s), {'from': 'small'})
    # Not straight back onto the link: the next attempt waits REJOIN_DELAY.
    self.assertGreater(s._rejoin_at, time.monotonic() + 1.0)
    self.assertTrue(s.loading)
    self.assertNotIn('ChestnutActive', self.params)

  def test_a_link_that_dies_before_the_swap_is_reopened(self):
    # A link waits in _joined until the frame loop finds a window, which on a
    # drive with no stop and no disengage is the whole drive. If the Jetson
    # reboots in there, finding out at the swap costs a frame and a demote.
    clients = []

    def connect():
      c = mock.MagicMock(name='client')
      clients.append(c)
      self.connect_calls += 1
      self.joined.set()
      return (c, 'spec')

    self._connect = connect
    with mock.patch('openpilot.sunnypilot.accelerators.jetlink.joining.KEEPALIVE_PERIOD', 0.05), \
         mock.patch('openpilot.sunnypilot.accelerators.jetlink.joining.REJOIN_DELAY', 0.05):
      s = self._state()
      self.assertTrue(self.joined.wait(5.0))
      # Never a window, so nothing consumes it; the ping is what notices.
      s._engaged, s._standstill = True, False
      clients[0].ping.side_effect = RuntimeError("jetson rebooted")
      for _ in range(100):
        if len(clients) > 1:
          break
        time.sleep(0.05)
      self.assertGreater(len(clients), 1, "a dead pending link was never reopened")
      clients[0].close.assert_called()
      # And the fresh one is what the window eventually gets.
      s._standstill = True
      for _ in range(40):
        if self._run(s) == {'from': 'big'}:
          break
        time.sleep(0.05)
      self.assertTrue(s.chestnut)

  def test_failures_back_off_and_a_stable_join_starts_over(self):
    # A link that dies on its first frame every time used to cost a swap, a
    # demote, a soft disable and a chime every REJOIN_DELAY for the drive.
    s = self._state()
    s._joined_at = 0.0
    delays = []
    for _ in range(5):
      t = time.monotonic()
      s._back_off()
      delays.append(round(s._rejoin_at - t))
    self.assertEqual(delays, [5, 10, 20, 40, 60])

    # A join that held is not that link, and must not inherit its delay: a
    # Jetson that reboots once an hour should be picked up in REJOIN_DELAY.
    s._joined_at = time.monotonic() - (STABLE_SECONDS + 1)
    t = time.monotonic()
    s._back_off()
    self.assertEqual(round(s._rejoin_at - t), 5)

  def test_small_model_failure_is_modelds(self):
    s = self._state()
    self.small.raises = RuntimeError("vipc gone")
    with self.assertRaises(RuntimeError):
      self._run(s)

  def test_lat_delay_reaches_both_models(self):
    s = self._state()
    s.lat_delay = 0.25
    self.assertEqual(self.small.lat_delay, 0.25)
    self.assertTrue(self.joined.wait(5.0))
    s._engaged = False
    self._run(s)
    self.assertEqual(self.big.lat_delay, 0.25)


  def test_reports_loading_until_it_joins(self):
    # The UI reads this to tell "not up yet" from "failed". modelV2.big is
    # false for the whole join, and without this the UI calls that a failure
    # and latches on it.
    s = self._state()
    self._run(s)
    self.assertIs(self.params.get('ChestnutLoading'), True)

    self.assertTrue(self.joined.wait(5.0))
    s._engaged = False
    self._run(s)
    self.assertIs(self.params.get('ChestnutLoading'), False)

    self.big.raises = RuntimeError("link gone")
    self._run(s)
    self.assertIs(self.params.get('ChestnutLoading'), True)


class ContractTest(unittest.TestCase):
  """Whatever modeld touches on the object make_model_state returns.

  Read out of modeld rather than kept by hand, because the list is modeld's and
  it has already been wrong once: `warmup` is called on the return value and
  nothing else, so it was missed, and modeld catches the AttributeError as
  "big model load failed" and spends the drive on the small model. A missing
  member has to fail here, not on the car.
  """

  def test_provides_everything_modeld_touches(self):
    src = Path(BASEDIR) / 'openpilot' / 'selfdrive' / 'modeld' / 'modeld.py'
    text = src.read_text()
    # `model` is the one in the frame loop, `m` the one load_big just built.
    # \b keeps small_model/big_model/sm out of it.
    names = set(re.findall(r'\bmodel\.([a-zA-Z_][a-zA-Z0-9_]*)', text))
    names |= set(re.findall(r'\bm\.([a-zA-Z_][a-zA-Z0-9_]*)', text))
    self.assertIn('warmup', names, "modeld stopped calling warmup; check this test still finds the right names")
    missing = sorted(n for n in names if not hasattr(JoiningModelState, n))
    self.assertEqual(missing, [], f"JoiningModelState is missing {missing}, which modeld calls on it")


if __name__ == '__main__':
  unittest.main()
