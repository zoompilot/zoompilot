"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Cruise panel badges, shown only where alpha long and ICBM split the cruise features between
# them. Everywhere else one system carries everything and a badge would say nothing:
#   speed limit      "icbm"            ICBM, not the planner's prompt, moves the set speed to
#                                      the limit (speed_limit.helpers.pcm_machine_owns_sla)
#   decel overshoot  "stock acc only"  greyed although ICBM is on; the planner brakes itself
# sunnylink cannot render per-car badges; cruise.yaml carries the same enablement.

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode as SpeedLimitMode
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.lib.styles import style

ICBM_BADGE = tr_noop("icbm")
STOCK_ACC_ONLY_BADGE = tr_noop("stock acc only")
STOCK_ACC_ONLY_REASON = tr_noop("Stock ACC only. zoompilot longitudinal control slows for curves itself.")


def icbm_moves_speed_limits(has_long: bool, has_icbm: bool, speed_limit_mode) -> bool:
  # only assist moves the set speed; info and warn leave it alone
  return has_long and has_icbm and speed_limit_mode == SpeedLimitMode.assist


class TiziCruiseBadges:
  """The tizi cruise layout's one hook. Rows carry the speed limit badge as right-aligned text;
  the overshoot reason is too long for its row, so it leads the description in bold instead, the
  way the ICBM row explains itself when unavailable."""

  def __init__(self, overshoot_item, speed_limit_item):
    self._overshoot = overshoot_item
    self._speed_limit = speed_limit_item
    self._overshoot_desc = overshoot_item.description
    self._shown_off = False
    self._reason = False

  def update(self, has_long: bool, has_icbm: bool, overshoot_available: bool) -> None:
    # longitudinal control brakes itself and ignores overshoot: show it off, but keep the stored
    # value for stock ACC (tizi toggles never re-read their param, so restore it on the way out)
    if has_long != self._shown_off:
      self._shown_off = has_long
      self._overshoot.action_item.set_state(not has_long and ui_state.params.get_bool("SmartCruiseDecelOvershoot"))

    reason = has_long and overshoot_available
    if reason != self._reason:
      self._reason = reason
      self._overshoot.set_description(f"<b>{tr(STOCK_ACC_ONLY_REASON)}</b>\n\n{self._overshoot_desc}" if reason else self._overshoot_desc)
      if reason:
        self._overshoot.show_description(True)

    icbm = icbm_moves_speed_limits(has_long, has_icbm, ui_state.speed_limit_mode)
    self._speed_limit.set_right_value(tr(ICBM_BADGE) if icbm else "", style.GREEN)
