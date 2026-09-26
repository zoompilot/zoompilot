"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import (TUNE_PARAM_BY_SIZE, jerk_aware_has_effect, label_for,
                                                                      stored_tune_versions, versions_by_label)
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.lib.utils import NoElideButtonAction
from openpilot.system.ui.sunnypilot.widgets.list_view import ListItemSP, toggle_item_sp, option_item_sp
from openpilot.system.ui.sunnypilot.widgets.tree_dialog import TreeOptionDialog, TreeFolder, TreeNode
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller



class TorqueSettingsLayout(Widget):
  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)
    self._torque_version_dialog: TreeOptionDialog | None = None
    self.cached_torque_versions = versions_by_label()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._jerk_aware_toggle = toggle_item_sp(
      param="LateralJerkTorqueController",
      title=lambda: tr("Lateral Jerk Torque Controller"),
      description=lambda: tr("Looks ahead at planned steering to reduce sudden corrections, so the wheel moves " +
                             "more smoothly through turns. Works with Self-Tune and custom tuning. " +
                             "Thanks to @twilsonco for the implementation."),
    )
    # one tune per model size; controlsd swaps them as modelV2.big changes
    def tune_row(big: bool, title: str, description: str) -> ListItemSP:
      return ListItemSP(title=title, description=description, action_item=NoElideButtonAction(tr("SELECT")),
                        callback=lambda: self._show_torque_version_dialog(big))
    self._tune_rows = {
      False: tune_row(False, tr("Torque Control Tune Version (Small Models)"),
                      tr("Select the version of Torque Control Tune to use while the small model is driving.")),
      True: tune_row(True, tr("Torque Control Tune Version (Big Models)"),
                     tr("Select the version of Torque Control Tune to use while a big model is driving.")),
    }
    self._self_tune_toggle = toggle_item_sp(
      param="LiveTorqueParamsToggle",
      title=lambda: tr("Self-Tune"),
      description=lambda: tr("Enables self-tune for Torque lateral control for platforms that do not use " +
                             "Torque lateral control by default."),
    )
    self._relaxed_tune_toggle = toggle_item_sp(
      param="LiveTorqueParamsRelaxedToggle",
      title=lambda: tr("Less Restrict Settings for Self-Tune (Beta)"),
      description=lambda: tr("Less strict settings when using Self-Tune. This allows torqued to be more " +
                             "forgiving when learning values."),
    )
    self._speed_dep_toggle = toggle_item_sp(
      param="SpeedDependentTorqueToggle",
      title=lambda: tr("Speed-Dependent Self-Tune (Beta)"),
      description=lambda: tr("Learns separate torque parameters at different speeds. " +
                             "Improves steering at both low and high speeds for supported cars."),
    )
    self._custom_tune_toggle = toggle_item_sp(
      param="CustomTorqueParams",
      title=lambda: tr("Enable Custom Tuning"),
      description=lambda: tr("Enables custom tuning for Torque lateral control. " +
                             "Modifying Lateral Acceleration Factor and Friction below will override the offline values " +
                             "indicated in the YAML files within \"opendbc/car/torque_data\". " +
                             "The values will also be used live when \"Manual Real-Time Tuning\" toggle is enabled."),
    )
    self._torque_prams_override_toggle = toggle_item_sp(
      param="TorqueParamsOverrideEnabled",
      title=lambda: tr("Manual Real-Time Tuning"),
      description=lambda: tr("Enforces the torque lateral controller to use the fixed values instead of the learned " +
                             "values from Self-Tune. Enabling this toggle overrides Self-Tune values."),
    )
    self._torque_lat_accel_factor = option_item_sp(
      title=lambda: tr("Lateral Acceleration Factor"),
      param="TorqueParamsOverrideLatAccelFactor",
      description="",
      min_value=1,
      max_value=500,
      value_change_step=1,
      label_callback=(lambda x: f"{x/100} m/s^2"),
      use_float_scaling=True
    )

    self._torque_friction = option_item_sp(
      title=lambda: tr("Friction"),
      param="TorqueParamsOverrideFriction",
      description="",
      min_value=1,
      max_value=100,
      value_change_step=1,
      label_callback=(lambda x: f"{x/100}"),
      use_float_scaling=True
    )

    items = [
      self._jerk_aware_toggle,
      *self._tune_rows.values(),
      self._self_tune_toggle,
      self._relaxed_tune_toggle,
      self._speed_dep_toggle,
      self._custom_tune_toggle,
      self._torque_prams_override_toggle,
      self._torque_lat_accel_factor,
      self._torque_friction,
    ]
    return items

  def _update_state(self):
    super()._update_state()
    nnlc_enabled = ui_state.params.get_bool("NeuralNetworkLateralControl")
    # v2 tune replaces the jerk-aware mechanisms and forces the controller off, so the
    # toggle is disabled while v2 is the tune every model size will actually run
    self._jerk_aware_toggle.action_item.set_enabled(ui_state.is_offroad() and not nnlc_enabled and jerk_aware_has_effect(ui_state.params))
    if not ui_state.params.get_bool("LiveTorqueParamsToggle"):
      ui_state.params.remove("LiveTorqueParamsRelaxedToggle")
      self._relaxed_tune_toggle.action_item.set_state(False)
    self._self_tune_toggle.action_item.set_enabled(ui_state.is_offroad())
    self._relaxed_tune_toggle.action_item.set_enabled(ui_state.is_offroad() and self._self_tune_toggle.action_item.get_state())
    self._speed_dep_toggle.set_visible(self._self_tune_toggle.action_item.get_state())
    self._speed_dep_toggle.action_item.set_enabled(ui_state.is_offroad())
    self._custom_tune_toggle.action_item.set_enabled(ui_state.is_offroad())
    custom_tune_enabled = self._custom_tune_toggle.action_item.get_state()
    self._torque_prams_override_toggle.set_visible(custom_tune_enabled)
    self._torque_lat_accel_factor.set_visible(custom_tune_enabled)
    self._torque_friction.set_visible(custom_tune_enabled)

    self._torque_prams_override_toggle.action_item.set_enabled(ui_state.is_offroad())
    sliders_enabled = self._torque_prams_override_toggle.action_item.get_state() or ui_state.is_offroad()
    self._torque_lat_accel_factor.action_item.set_enabled(sliders_enabled)
    self._torque_friction.action_item.set_enabled(sliders_enabled)

    title_text = tr("Real-Time & Offline") if ui_state.params.get("TorqueParamsOverrideEnabled") else tr("Offline Only")
    self._torque_lat_accel_factor.set_title(lambda: tr("Lateral Acceleration Factor") + " (" + title_text + ")")
    self._torque_friction.set_title(lambda: tr("Friction") + " (" + title_text + ")")
    for big, version in stored_tune_versions(ui_state.params).items():
      self._tune_rows[big].action_item.set_value(self._version_label(version))

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()
    # subtract button
    content_rect = rl.Rectangle(rect.x, rect.y + self._back_button.rect.height + 40, rect.width, rect.height - self._back_button.rect.height - 40)
    self._scroller.render(content_rect)

  def show_event(self):
    self._scroller.show_event()

  def _version_label(self, version: float) -> str:
    # the stored value resolves through the declared param default, the same read
    # controlsd_ext makes: a "Default" placeholder would hide which tune the car actually runs
    return label_for(version, self.cached_torque_versions) or tr("Unknown")

  def _show_torque_version_dialog(self, big: bool):
    options_map = self.cached_torque_versions
    # newest first
    sorted_labels = sorted(options_map, key=lambda k: options_map[k], reverse=True)
    folders = [TreeFolder("", [TreeNode(label) for label in sorted_labels])]
    current_label = self._version_label(stored_tune_versions(ui_state.params)[big])

    def handle_selection(result: int):
      if result == DialogResult.CONFIRM and self._torque_version_dialog:
        selected_ref = self._torque_version_dialog.selection_ref
        if selected_ref in options_map:
          ui_state.params.put(TUNE_PARAM_BY_SIZE[big], options_map[selected_ref])
      self._torque_version_dialog = None

    self._torque_version_dialog = TreeOptionDialog(
      tr("Select Torque Control Tune Version"),
      folders,
      current_ref=current_label,
      option_font_weight=FontWeight.UNIFONT,
      on_exit=handle_selection,
    )
    gui_app.push_widget(self._torque_version_dialog)
