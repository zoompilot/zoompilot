"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Onroad events for an accelerator that joins mid-drive: jetlink's swap events
(jetlink/openpilot/events.py, which says why each one is there, the
2026-10-04 TAKE CONTROL IMMEDIATELY among them) read off this fork's messages,
and their flags raised as this fork's events. An enable inside the no-entry
second is not lost: cruise is refused and pressed again after the chime, and
MADS goes to paused and resumes by itself (mads/state.py). With no jetlink on
the device there are none.
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName
EventNameZP = custom.OnroadEventZP.EventName

# jetlink's lengths (events.OFFER_S, HANDBACK_S, SWITCHING_S) in selfdrived's
# ticks, for the tests and traces; written out, as selfdrived starts without jetlink
OFFER_TICKS = round(3. / DT_CTRL)
HANDBACK_TICKS = round(5. / DT_CTRL)
SWITCHING_TICKS = round(1. / DT_CTRL)


class AcceleratorEvents:
  # An accelerator is optional: its daemon exiting costs the big model, never
  # engagement. manager does not restart a process that died, so without this
  # a dead link owner would be processNotRunning, NO_ENTRY until a reboot
  OPTIONAL_PROCESSES = frozenset({jetlink_adapter.OWNER})

  def __init__(self):
    # jetlink's tracker, behind the hooks' net; None without jetlink
    self.swap = jetlink_adapter.swap_events(DT_CTRL)

  @property
  def settling(self) -> bool:
    """Within a second of the big model swapping in or handing back."""
    return self.swap is not None and self.swap.settling

  def update(self, sm: messaging.SubMaster, in_control: bool, events: Events, events_sp: EventsSP) -> None:
    """`in_control`: openpilot or MADS is engaged, MADS even while its lateral
    is paused, the rule the adapter's swap gate waits on (Adapter.engaged)."""
    if self.swap is None:
      return
    fresh = all(sm.seen[s] and sm.alive[s] and sm.valid[s] for s in ('modelV2', 'modelDataV2SP'))
    # modelV2's own liveness: a late modelDataV2SP is no hand-back. A chestnut
    # dropping modelV2.big is the native bigModelFailed; its state is none
    big = sm.alive['modelV2'] and sm.valid['modelV2'] and sm['modelV2'].big
    flags = self.swap.update(fresh, big, in_control, state=str(sm['modelDataV2SP'].zoompilot.acceleratorState))
    if flags.offer:
      events_sp.zp.add(EventNameZP.bigModelAvailable)
    if flags.link_lost:
      events_sp.zp.add(EventNameZP.bigModelLinkLost)
    if flags.no_entry or flags.chime:
      # the native block's chime on the swap waits for the end of the second,
      # when it means the driver can engage
      events_sp.remove(EventNameSP.bigModelReady)
    if flags.no_entry:
      # upstream's no-entry for a big model that is not ready to drive.
      # Native, since the main state machine reads only native events
      events.add(EventName.bigModelLoading)
    if flags.chime:
      events_sp.add(EventNameSP.bigModelReady)
