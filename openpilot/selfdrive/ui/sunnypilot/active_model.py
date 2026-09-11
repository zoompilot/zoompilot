"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import re

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.models.default_model import get_default_model
from openpilot.sunnypilot.models.helpers import get_active_bundle

RELEASE_DATE = re.compile(r"\s*\([^()]*\)\s*$")

_offroad_name: str | None = None


def active_model_name() -> str:
  """The driving model modeld is running, without its release date.

  models_manager only runs offroad, so a UI that came up onroad never receives modelManagerSP.
  There the name comes from the same ModelManager_ActiveBundle param modeld itself reads,
  instead of falling back to the default name and confidently naming the wrong model.
  """
  global _offroad_name

  if ui_state.sm.recv_frame["modelManagerSP"] > 0:
    bundle = ui_state.sm["modelManagerSP"].activeBundle
    name = bundle.displayName if bundle.ref else get_default_model()
  else:
    if _offroad_name is None:
      bundle = get_active_bundle(ui_state.params)
      _offroad_name = bundle.displayName if bundle else get_default_model()
    name = _offroad_name

  return RELEASE_DATE.sub("", name)
