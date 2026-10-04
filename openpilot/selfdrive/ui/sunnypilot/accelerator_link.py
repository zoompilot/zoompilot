"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The user's say over the Jetlink setting, shared by the mici and tici models panels:
Off, USB (a Jetson, a Linux PC or a Mac) or iOS (an iPhone), stored as an
index into the adapter's MODES. The small model is picked as ever: manager
runs whichever modeld that bundle needs and the link joins it, so the setting
never changes which modeld runs. Everything else here reads ui_state.jetlink,
the snapshot the params pass takes.
"""
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.jetlink_adapter import KEYS, MODES
from openpilot.system.ui.lib.multilang import tr

LINK_MODES = MODES
LINK_PARAM = KEYS.link
LINK_MODE_TITLES = {"off": "Off", "usb": "USB", "ios": "iOS"}


def link_mode() -> str:
  """The setting as stored, one of LINK_MODES: the panel shows it even when
  jetlink cannot run, so a link that will not start can be turned off."""
  index = ui_state.params.get(LINK_PARAM, return_default=True)
  return LINK_MODES[index] if isinstance(index, int) and 0 <= index < len(LINK_MODES) else "off"


def link_toggle_meaningful() -> bool:
  """Offered wherever jetlink is checked out, as the chestnut slot is offered
  whether or not a board is fitted, except beside a chestnut, which runs the
  big model itself; and while the setting is on, whatever else. Presence
  cannot be the gate: with the link off there is no gadget for a Jetson to
  enumerate, so the toggle that turns it on would wait for the thing it enables."""
  if ui_state.chestnut_present:
    return False
  return ui_state.jetlink is not None or link_mode() != "off"


def link_status() -> str:
  """One line under the toggle: what is on the comma's USB-C port right now.

  jetlink knows a Jetson, or a phone, and the transport says which. Below that
  only the CC pin speaks: it says a cable with a host behind it is plugged in,
  not what the host is. Empty where the kernel does not expose it, rather
  than claiming an empty port.
  """
  jetlink = ui_state.jetlink
  if jetlink is None:
    return ""
  if jetlink.present:
    return f"{tr('Jetlink connected:')} {jetlink.transport}."
  if jetlink.port is None:
    return ""
  return tr("Nothing on the USB port.") if jetlink.port == "empty" else tr("A device is on the USB port.")
