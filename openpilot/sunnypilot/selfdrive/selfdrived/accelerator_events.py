"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Onroad events for an accelerator that joins mid-drive. The native big model block
expects a board loaded before the first modelV2; an off-board one joins onto a
modelV2 the small model already publishes and can leave and come back.

It swaps in only while nothing is in control, so the driver is told when it is
ready and re-engages to use it. For a second after a swap nothing engages while
the large model builds its history and proves it keeps up (jetlink hands back
on a held frame in that second), and the "Big Model Active" chime at the end of
it says the driver can. When it leaves, the small model drives on and the
driver is told it is on the small model; nothing disengages.

Either way the switch costs modeld a frame or two, and a frame the camera
dropped is an invalid pose from the model and a locationd soft disable on the
next tick: every return of a user's big model was a TAKE CONTROL IMMEDIATELY
(2026-10-04). So a switch is `settling` for a second, which selfdrived treats
as it does a native board's load (big_model_settling).
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName
AcceleratorState = custom.ModelDataV2SP.AcceleratorState

# how long each event is raised for, in selfdrived's ticks. The take-control
# warning covers about the time the small model takes to refill its history;
# the no-entry after a swap, about 20 frames of the large model's. An enable
# inside that second is not lost: cruise is refused and pressed again after
# the chime, and MADS goes to paused and resumes by itself (mads/state.py)
OFFER_TICKS = round(3. / DT_CTRL)
HANDBACK_TICKS = round(5. / DT_CTRL)
SWITCHING_TICKS = round(1. / DT_CTRL)


class AcceleratorEvents:
  # An accelerator is optional: its daemon exiting costs the big model, never
  # engagement. manager does not restart a process that died, so without this
  # a dead link owner was processNotRunning, NO_ENTRY until a reboot
  OPTIONAL_PROCESSES = frozenset({jetlink_adapter.OWNER})

  def __init__(self):
    self.offered = False
    self.big_model_running = False
    # ticks left of each event, and of the settling after a switch either way
    self.offer = self.handback = self.switching = self.settle = 0

  @property
  def settling(self) -> bool:
    """Within a second of the big model swapping in or handing back."""
    return self.settle > 0

  def update(self, sm: messaging.SubMaster, in_control: bool, events: Events, events_sp: EventsSP) -> None:
    """`in_control`: openpilot or MADS is engaged, MADS even while its lateral
    is paused. The adapter's swap gate is shut exactly then."""
    status = sm['modelDataV2SP']
    big = sm['modelV2'].big

    # stale status neither offers the switch nor rearms the offer
    if all(sm.seen[s] and sm.alive[s] and sm.valid[s] for s in ('modelV2', 'modelDataV2SP')):
      if status.acceleratorState != AcceleratorState.ready or big:
        # ended by the swap as well as by the link going: an offer still on
        # screen after the switch read as if it had not happened
        self.offered = False
        self.offer = 0
      elif in_control and not self.offered and self.handback == 0:
        # with nothing in control it swaps in at once and bigModelReady says
        # so. Once per readiness, not at every stop, and after the take-control
        # warning, which would hide it
        self.offered = True
        self.offer = OFFER_TICKS

    # a chestnut dropping modelV2.big raises the native bigModelFailed
    running_big = sm.alive['modelV2'] and sm.valid['modelV2'] and big and \
      status.acceleratorState != AcceleratorState.none
    if running_big and not self.big_model_running:
      self.switching = SWITCHING_TICKS + 1   # the no-entry, then the chime
      self.settle = SWITCHING_TICKS + 1   # counted down below, this tick included
    elif self.big_model_running and not running_big:
      self.switching = 0
      self.settle = SWITCHING_TICKS + 1   # counted down below, this tick included
      if in_control:
        self.handback = HANDBACK_TICKS
    self.big_model_running = running_big
    if self.settle > 0:
      self.settle -= 1
    if not in_control:
      self.handback = 0

    if self.offer > 0:
      self.offer -= 1
      events_sp.add(EventNameSP.bigModelAvailable)
    if self.handback > 0:
      self.handback -= 1
      events_sp.add(EventNameSP.bigModelLinkLost)
    if self.switching > 0:
      self.switching -= 1
      # the native block's chime on the swap waits for the end of the second,
      # when it means the driver can engage
      events_sp.remove(EventNameSP.bigModelReady)
      if self.switching:
        # upstream's no-entry for a big model that is not ready to drive.
        # Native, since the main state machine reads only native events
        events.add(EventName.bigModelLoading)
      else:
        events_sp.add(EventNameSP.bigModelReady)
