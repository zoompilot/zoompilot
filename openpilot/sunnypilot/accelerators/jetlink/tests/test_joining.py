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

from openpilot.sunnypilot.accelerators.jetlink.joining import JoiningModelState


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

  def test_does_not_swap_while_engaged(self):
    s = self._state()
    self.assertTrue(self.joined.wait(5.0))
    s._engaged = True
    for _ in range(3):
      self.assertEqual(self._run(s), {'from': 'small'})
    self.assertFalse(s.chestnut)
    self.assertFalse(self.big.warmed)

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

  def test_loading_alert_ends_after_the_timeout_but_the_join_does_not(self):
    self.connect_error = RuntimeError("jetson still booting")
    s = self._state()
    self._run(s)
    self.assertTrue(self.params.get("ChestnutLoading"))
    with mock.patch('openpilot.sunnypilot.accelerators.jetlink.joining.time.monotonic',
                    return_value=time.monotonic() + 61.0):
      self._run(s)
    self.assertFalse(self.params.get("ChestnutLoading"))
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
