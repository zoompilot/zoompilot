"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamOption
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.display import ONROAD_BRIGHTNESS_TIMER_VALUES, OnroadBrightness
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


def _timer_picker_unit():
  raw = ui_state.params.get("OnroadScreenOffTimer", return_default=True) or 0
  mapped = ONROAD_BRIGHTNESS_TIMER_VALUES.get(raw, raw)
  return tr("seconds") if mapped < 60 else tr("minutes")


def _brightness_label(val):
  if val == OnroadBrightness.AUTO:
    return tr("auto")
  if val == OnroadBrightness.AUTO_DARK:
    return tr("auto (dark)")
  if val == OnroadBrightness.SCREEN_OFF:
    return tr("screen off")
  return f"{(val - 2) * 5}%"


def _brightness_picker_label(val):
  if val == OnroadBrightness.AUTO:
    return tr("auto")
  if val == OnroadBrightness.AUTO_DARK:
    return tr("auto") + "\n" + tr("(dark)")
  if val == OnroadBrightness.SCREEN_OFF:
    return "0\n" + tr("(off)")
  return str((val - 2) * 5)


def _timer_label(val):
  if val < 60:
    return f"{val} " + (tr("seconds") if val != 1 else tr("second"))
  mins = int(val / 60)
  return f"{mins} " + (tr("minutes") if mins != 1 else tr("minute"))


def _timer_picker_label(val):
  if val < 60:
    return str(val)
  return str(int(val / 60))


def _timeout_label(val):
  if not val:
    return tr("default")
  return f"{val} " + tr("seconds")


class DisplayLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._brightness = BigParamOption(
      tr("brightness"), "OnroadScreenOffBrightness",
      min_value=0, max_value=22,
      label_callback=_brightness_label,
      picker_label_callback=_brightness_picker_label,
      picker_unit="%",
      picker_item_width=140,
    )
    self._brightness_timer = BigParamOption(
      tr("brightness delay"), "OnroadScreenOffTimer",
      min_value=0, max_value=15,
      value_map=ONROAD_BRIGHTNESS_TIMER_VALUES,
      label_callback=_timer_label,
      picker_label_callback=_timer_picker_label,
      picker_unit=_timer_picker_unit,
    )
    self._ui_timeout = BigParamOption(
      tr("ui timeout"), "InteractivityTimeout",
      min_value=0, max_value=120, value_change_step=10,
      label_callback=_timeout_label,
      picker_label_callback=lambda v: tr("default") if not v else str(v),
      picker_unit=tr("seconds"),
    )

    self._screensaver = BigParamControl(tr("screen saver"), "ScreenSaverEnabled")
    # Match the TICI screen-saver range and one-minute step.
    self._screensaver_timeout = BigParamOption(
      tr("saver duration"), "ScreenSaverTimeout",
      min_value=60, max_value=600, value_change_step=60,
      label_callback=_timer_label,
      picker_label_callback=_timer_picker_label,
      picker_unit=tr("minutes"),
    )

    self._scroller.add_widgets([self._brightness, self._brightness_timer, self._ui_timeout,
                                self._screensaver, self._screensaver_timeout])

  def _update_state(self):
    super()._update_state()
    self._brightness.refresh()
    self._brightness_timer.refresh()
    self._ui_timeout.refresh()
    self._screensaver.refresh()
    self._screensaver_timeout.refresh()

    brightness_val = ui_state.params.get("OnroadScreenOffBrightness", return_default=True)
    self._brightness_timer.set_enabled(
      brightness_val not in (OnroadBrightness.AUTO, OnroadBrightness.AUTO_DARK)
    )
    # gated like the brightness timer above; the param keeps its value while the toggle is off
    self._screensaver_timeout.set_enabled(self._screensaver._checked)
