"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace
from unittest.mock import Mock

from openpilot.cereal import messaging
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.sunnypilot.selfdrive.selfdrived.model_events import ModelEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SERVICES = ['modelV2', 'modelDataV2SP', 'controlsState', 'deviceState', 'lateralManeuverPlan', 'alertDebug']


def make_selfdrived(chestnut_present: bool = False, enabled: bool = False, mads_enabled: bool = False) -> SelfdriveD:
  """A SelfdriveD whose update_events runs as far as its initialization gate:
  the big model block and the accelerator adapter, with no processes, no live
  params and no car. `mads_enabled` is MADS engaged, which the adapter reads
  beside `enabled`.

  Built without __init__, so every attribute that path reads is set here, and
  a new one breaks this function rather than every suite that drives it.
  modelV2 and modelDataV2SP are writable and valid but not yet seen: the
  native block reads a board as failed only once a modelV2 it had has gone
  away, so each suite says when they arrive. The params answer as a device
  that has loaded nothing.
  """
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.sm = messaging.SubMaster(SERVICES)
  for service in ('modelV2', 'modelDataV2SP', 'deviceState'):
    sd.sm.data[service] = sd.sm[service].as_builder()
    sd.sm.valid[service] = True
  sd.sm.seen['deviceState'] = sd.sm.alive['deviceState'] = True
  sd.sm['deviceState'].chestnutPresent = chestnut_present
  sd.events = Events()
  sd.events_sp = EventsSP()
  sd.model_events = ModelEventsSP()
  sd.params = Mock()
  sd.params.get_bool.return_value = False
  sd.params.get.return_value = None
  sd.big_model_loading = sd.big_model_active = sd.big_model_failed = False
  sd.big_model_ready_t = 0.
  sd.enabled = enabled
  sd.mads = SimpleNamespace(enabled=mads_enabled)
  sd.initialized = False
  sd.startup_event = None
  return sd
