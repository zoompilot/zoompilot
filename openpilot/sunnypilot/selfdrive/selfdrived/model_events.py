"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

selfdrived's model events in one call: the chime on the first big frame,
an accelerator that joins mid-drive (accelerator_events.py) and modeld's
first load on every boot (model_startup.py).
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.sunnypilot.selfdrive.selfdrived.accelerator_events import AcceleratorEvents
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.model_startup import ModelStartup

EventNameSP = custom.OnroadEventSP.EventName


class ModelEventsSP:
  OPTIONAL_PROCESSES = AcceleratorEvents.OPTIONAL_PROCESSES

  def __init__(self):
    self.accelerator = AcceleratorEvents()
    self.startup = ModelStartup()
    self.big_model_running = False

  @property
  def settling(self) -> bool:
    """selfdrived holds its comm checks as it does through a big model load: for
    a second after a jetlink switch either way, and through modeld's first load."""
    return self.accelerator.settling or self.startup.starting

  def update(self, sm: messaging.SubMaster, in_control: bool, initialized: bool, big_model_loading: bool,
             events: Events, events_sp: EventsSP) -> None:
    """Runs just before selfdrived's initialization gate, so on every frame."""
    # upstream chimes when ChestnutLoading clears, which it also does after a
    # failed load, so only modelV2.big means the big model is up
    events_sp.remove(EventNameSP.bigModelReady)
    running_big = sm.alive['modelV2'] and sm.valid['modelV2'] and sm['modelV2'].big
    if running_big and not self.big_model_running:
      events_sp.add(EventNameSP.bigModelReady)
    self.big_model_running = running_big

    self.accelerator.update(sm, in_control, events, events_sp)

    # modeld's first load outlasts selfdrived's 6 s initialization
    if initialized and self.startup.update(sm) and not big_model_loading:
      events.add(EventName.bigModelLoading)
