"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The user's say over the accelerator link, shared by the mici and tici models panels.
On is the only enable; the backend reads the param, the panels only write it. The
small model is picked as ever: manager runs whichever modeld that bundle needs and
the accelerator joins it, so the toggle never changes which modeld runs.
"""
from openpilot.common.hardware.usb import TYPEC_CC_ORIENTATION_PATH, read
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot import accelerators
from openpilot.system.ui.lib.multilang import tr

LINK_PARAM = "JetlinkEnabled"


def link_enabled() -> bool:
  """never raises: a params library older than the key would take the settings panel down"""
  try:
    return bool(ui_state.params.get(LINK_PARAM))
  except Exception:
    return False


def set_link_enabled(enabled: bool) -> None:
  try:
    ui_state.params.put_bool(LINK_PARAM, enabled, block=True)
  except Exception:
    pass  # same unknown-key case as the read


def link_toggle_meaningful() -> bool:
  """Offered wherever the package is checked out, as the chestnut slot is offered
  whether or not a board is fitted, except beside a chestnut, which runs the big
  model itself. Otherwise hidden only on a device with nothing of ours: no
  package, nothing attached, nothing set, no complaint. present() cannot be the
  gate on its own: with the link off there is no gadget for a Jetson to enumerate,
  so the toggle that turns it on would wait for the thing it enables."""
  if ui_state.chestnut_present:
    return False
  return (accelerators.installed() or accelerators.present() or accelerators.ready() or link_enabled()
          or accelerators.unavailable_reason() is not None)


def link_status() -> str:
  """One line under the toggle: what is on the comma's USB-C port right now.

  present() knows a Jetson. Below that only the CC pin speaks: it says a cable
  with a host behind it is plugged in, not what the host is. Empty where the
  kernel does not expose it, rather than claiming an empty port.
  """
  if accelerators.present():
    return tr("Accelerator connected.")
  raw = read(TYPEC_CC_ORIENTATION_PATH)
  if raw is None:
    return ""
  return tr("Nothing on the USB port.") if raw == "0" else tr("A device is on the USB port.")

