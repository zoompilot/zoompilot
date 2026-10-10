"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Helpers the tici and mici settings tests share; the window and params
fixtures are in ../conftest.py.
"""


def render(widget, width: int = 800, height: int = 600):
  """Drive one frame through Widget.render, which calls _update_state."""
  import pyray as rl
  return widget.render(rl.Rectangle(0, 0, width, height))


def jetlink_status(**fields):
  """jetlink's snapshot as the UI's params pass takes it, nothing to show unless
  a field says so. A link turned on reads as USB unless the mode is given."""
  from jetlink.openpilot import Status
  base = {'enabled': False, 'mode': 'usb' if fields.get('enabled') else 'off', 'transport': 'USB', 'present': False,
          'port': None, 'ready': False, 'reason': None, 'progress': None, 'model': None, 'default_model': None}
  return Status(**{**base, **fields})
