"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Every event a big model raises, in order, for three drives.

Asserted as the whole list at every step, native and sunnypilot together,
because the failures that reached the car were extra events: a "Big Model
Ready" chime after a failed load, the native bigModelFailed (upstream's
"Restart the car to retry") on a jetlink drive, and an offer to switch shown
again after the switch.

Three traces, all through the real SelfdriveD.update_events with the
accelerator adapter in place:

  (a) a chestnut that loads:   loading, then one chime on the first big frame
  (b) a chestnut that fails:   loading, one failure, and never a chime
  (c) a jetlink that joins:    the offer to switch once while engaged, one
                               chime when it swaps in with nobody in control
                               and a second of no-entry, and on a fall while
                               engaged the take-control warning for 5 s and
                               nothing native: the small model drives on

update_events runs as far as its initialization gate, past the big model
block and the adapter call and short of everything needing a car.
"""
import unittest
from types import SimpleNamespace

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import EVENT_NAME
from openpilot.sunnypilot.selfdrive.selfdrived.accelerator_events import HANDBACK_TICKS, OFFER_TICKS, SWITCHING_TICKS
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENT_NAME_SP
from openpilot.sunnypilot.selfdrive.selfdrived.tests.selfdrived_helpers import make_selfdrived

AcceleratorState = custom.ModelDataV2SP.AcceleratorState

# added at the gate on every step, so it is in every expected list rather
# than filtered out
INIT = 'selfdriveInitializing'


class TraceTest(OpenpilotTestCase):
  def step(self, loading=False, active=None, big=False, alive=True,
           state=AcceleratorState.none, standstill=False) -> tuple[list[str], list[str]]:
    """One update_events, and everything it raised."""
    sd = self.sd
    sd.params.get_bool.return_value = loading
    sd.params.get.return_value = active
    # modelV2 and modelDataV2SP are published together; the native block reads
    # a board as failed only once a modelV2 it had has gone away
    for service in ('modelV2', 'modelDataV2SP'):
      sd.sm.alive[service] = alive
      sd.sm.seen[service] = sd.sm.seen[service] or alive
    sd.sm['modelV2'].big = big
    sd.sm['modelDataV2SP'].acceleratorState = state
    # carState reaches selfdrived on its own socket, not through the SubMaster
    sd.update_events(SimpleNamespace(standstill=standstill, canValid=False))
    return ([EVENT_NAME[n] for n in sd.events.names], [EVENT_NAME_SP[n] for n in sd.events_sp.names])


class ChestnutTraces(TraceTest):
  """comma's board, which is loaded before modelV2 exists and never rejoins."""

  def setUp(self):
    self.sd = make_selfdrived(chestnut_present=True, enabled=True)

  def test_trace_a_a_load_that_works_chimes_once(self):
    # modeld holds modelV2 back for the load, so the driver is kept out.
    self.assertEqual(self.step(loading=True, alive=False), ([INIT, 'bigModelLoading'], []))
    # ChestnutLoading clears, but nothing big has been published yet.
    self.assertEqual(self.step(loading=False, active=True, alive=False), ([INIT], []))
    # The first big frame, and the only chime in the drive.
    self.assertEqual(self.step(loading=False, active=True, big=True), ([INIT], ['bigModelReady']))
    for _ in range(10):
      self.assertEqual(self.step(loading=False, active=True, big=True), ([INIT], []))

  def test_trace_b_a_load_that_fails_never_chimes(self):
    self.assertEqual(self.step(loading=True, alive=False), ([INIT, 'bigModelLoading'], []))
    # modeld writes ChestnutActive=False first: the load timed out or threw.
    self.assertEqual(self.step(loading=True, active=False, alive=False), ([INIT, 'bigModelLoading', 'bigModelFailed'], []))
    # and clears ChestnutLoading once the small model is up; that second edge
    # used to chime "Big Model Ready" over "Big Model Failed"
    self.assertEqual(self.step(loading=False, active=False), ([INIT], []))
    for _ in range(10):
      self.assertEqual(self.step(loading=False, active=False), ([INIT], []))

  def test_a_board_that_falls_back_mid_drive_is_the_native_failure_alone(self):
    self.assertEqual(self.step(loading=False, active=True, big=True), ([INIT], ['bigModelReady']))
    # The adapter must not double this: acceleratorState is none for a board.
    self.assertEqual(self.step(loading=False, active=False, big=False), ([INIT, 'bigModelFailed'], []))


class JetlinkTrace(TraceTest):
  """A Jetson on its own power: it joins onto a modelV2 the small model owns.

  Nothing in this trace writes ChestnutLoading or ChestnutActive - the joining
  state never does - so the native block sees a device with no board and
  stays silent for the whole drive. Everything said is said by the adapter,
  from modelV2.big and modelDataV2SP.acceleratorState.
  """

  def setUp(self):
    self.sd = make_selfdrived(chestnut_present=False, enabled=True)

  def engage(self, enabled=False, mads=False):
    self.sd.enabled, self.sd.mads.enabled = enabled, mads

  def steps(self, n, **kwargs):
    return [self.step(**kwargs) for _ in range(n)]

  def test_trace_c_join_switch_and_lose_the_link(self):
    # modelV2 publishes on the small model from the first frame; the join
    # carries on in the background and blocks nothing
    self.assertEqual(self.step(state=AcceleratorState.joining), ([INIT], []))
    # ready while engaged: the offer, once, for its three seconds
    ready = self.steps(OFFER_TICKS + 200, state=AcceleratorState.ready)
    self.assertEqual(ready[:OFFER_TICKS], [([INIT], ['bigModelAvailable'])] * OFFER_TICKS)
    # and not again, stopped or not
    self.assertEqual(set(map(str, ready[OFFER_TICKS:])), {str(([INIT], []))})
    self.assertEqual(self.step(state=AcceleratorState.ready, standstill=True), ([INIT], []))
    # the driver turns everything off: it swaps. A second in which nothing
    # engages while the large model builds its history, then one chime, which
    # now means the driver can engage
    self.engage()
    swap = self.steps(SWITCHING_TICKS + 10, state=AcceleratorState.running, big=True)
    self.assertEqual(swap[:SWITCHING_TICKS], [([INIT, 'bigModelLoading'], [])] * SWITCHING_TICKS)
    self.assertEqual(swap[SWITCHING_TICKS], ([INIT], ['bigModelReady']))
    self.assertEqual(set(map(str, swap[SWITCHING_TICKS + 1:])), {str(([INIT], []))})
    # re-engaged on the large model, the link drops: the warning, and nothing
    # native, so neither state machine is told to disengage
    self.engage(enabled=True, mads=True)
    self.assertEqual(self.step(state=AcceleratorState.running, big=True), ([INIT], []))
    lost = self.steps(HANDBACK_TICKS + 10, state=AcceleratorState.retrying)
    self.assertEqual(lost[:HANDBACK_TICKS], [([INIT], ['bigModelLinkLost'])] * HANDBACK_TICKS)
    self.assertEqual(set(map(str, lost[HANDBACK_TICKS:])), {str(([INIT], []))})
    # it comes back, which a chestnut never does, and waits for the next window
    self.assertEqual(self.step(state=AcceleratorState.ready), ([INIT], ['bigModelAvailable']))

  def test_mads_on_at_a_stop_keeps_it_waiting(self):
    # lateral paused at a standstill is still MADS engaged: the offer, and no
    # swap until the driver turns it off (the adapter's gate reads the same)
    self.engage(mads=True)
    self.assertEqual(self.step(state=AcceleratorState.ready, standstill=True), ([INIT], ['bigModelAvailable']))

  def test_a_mads_only_loss_warns(self):
    self.engage(mads=True)
    self.steps(SWITCHING_TICKS + 1, state=AcceleratorState.running, big=True)
    self.assertEqual(self.step(state=AcceleratorState.retrying), ([INIT], ['bigModelLinkLost']))

  def test_a_loss_with_nothing_in_control_says_nothing(self):
    self.engage()
    self.steps(SWITCHING_TICKS + 1, state=AcceleratorState.running, big=True)
    self.assertEqual(self.steps(10, state=AcceleratorState.retrying), [([INIT], [])] * 10)

  def test_nothing_is_said_on_a_device_with_no_accelerator_at_all(self):
    for _ in range(10):
      self.assertEqual(self.step(), ([INIT], []))


if __name__ == '__main__':
  unittest.main()
