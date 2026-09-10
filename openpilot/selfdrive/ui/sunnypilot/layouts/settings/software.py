"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import subprocess

from openpilot.selfdrive.ui.layouts.settings.software import SoftwareLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.common.api.backend import is_konik_locked, put_bool_checked, set_konik_enabled, use_konik
from openpilot.common.hardware import HARDWARE
from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog

from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from openpilot.system.ui.sunnypilot.widgets.tree_dialog import TreeOptionDialog, TreeNode, TreeFolder


DESCRIPTIONS = {
  'disable_updates_offroad': tr_noop(
    "When enabled, automatic software updates will be off.<br><b>This requires a reboot to take effect.</b>"
  ),
  'disable_updates_onroad': tr_noop(
    "Please enable \"Always Offroad\" mode or turn off the vehicle to adjust these toggles."
  ),
  'konik_unlocked': tr_noop(
    "Send Connect registration, Athena, and route uploads to stable.konik.ai instead of comma.ai. A reboot is required."
  ),
  'konik_locked': tr_noop(
    "Locked on because Torque Interceptor was enabled. A factory reset is required to use comma.ai again."
  ),
}


class SoftwareLayoutSP(SoftwareLayout):
  def __init__(self):
    super().__init__()
    self.disable_updates_toggle = toggle_item_sp(
      lambda: tr("Disable Updates"),
      description="",
      initial_state=ui_state.params.get_bool("DisableUpdates"),
      callback=self._on_disable_updates_toggled,
    )
    self.konik_toggle = toggle_item_sp(
      lambda: tr("Konik Stable Connect"),
      description="",
      initial_state=use_konik(ui_state.params),
      callback=self._on_konik_toggled,
    )
    self._scroller.add_widget(self.konik_toggle)
    self._scroller.add_widget(self.disable_updates_toggle)

  def _restore_konik_toggle(self):
    self.konik_toggle.action_item.set_state(use_konik(ui_state.params))

  def _on_konik_toggled(self, enabled):
    if not enabled and is_konik_locked(ui_state.params):
      self._restore_konik_toggle()
      return

    def handle_reboot(result):
      try:
        if result != DialogResult.CONFIRM or not ui_state.is_offroad():
          return
        set_konik_enabled(ui_state.params, enabled)
        try:
          put_bool_checked(ui_state.params, "DoReboot", True)
        except Exception:
          set_konik_enabled(ui_state.params, not enabled)
          raise
      except Exception:
        cloudlog.exception("Failed to change Connect backend")
        gui_app.push_widget(alert_dialog(tr("Failed to save Connect backend change. Please try again.")))
      finally:
        self._restore_konik_toggle()

    dialog = ConfirmDialog(tr("System reboot required for changes to take effect. Reboot now?"), tr("Reboot"), callback=handle_reboot)
    gui_app.push_widget(dialog)

  def _handle_reboot(self, result):
    if result == DialogResult.CONFIRM:
      ui_state.params.put_bool("DisableUpdates", self.disable_updates_toggle.action_item.get_state())
      ui_state.params.put_bool("DoReboot", True)
    else:
      self.disable_updates_toggle.action_item.set_state(ui_state.params.get_bool("DisableUpdates"))

  def _on_disable_updates_toggled(self, enabled):
    dialog = ConfirmDialog(tr("System reboot required for changes to take effect. Reboot now?"), tr("Reboot"), callback=self._handle_reboot)
    gui_app.push_widget(dialog)

  def _on_select_branch(self):
    current_git_branch = ui_state.params.get("GitBranch") or ""
    branches_str = ui_state.params.get("UpdaterAvailableBranches") or ""
    branches = [b for b in branches_str.split(",") if b]
    current_target = ui_state.params.get("UpdaterTargetBranch") or ""
    top_level_branches = [current_git_branch, "release-mici", "release-tizi", "staging", "dev", "master"]

    if HARDWARE.get_device_type() == "tici":
      top_level_branches = ["release-tici", "staging-tici"]
      branches = [b for b in branches if b.endswith("-tici")]

    top_level_nodes = [TreeNode(b, {'display_name': b}) for b in top_level_branches if b in branches]
    remaining_branches = [b for b in branches if b not in top_level_branches]
    prebuilt_nodes = [TreeNode(b, {'display_name': b}) for b in remaining_branches if b.endswith("-prebuilt")]
    non_prebuilt_nodes = [TreeNode(b, {'display_name': b}) for b in remaining_branches if not b.endswith("-prebuilt")]

    folders = [
      TreeFolder("", top_level_nodes),
      TreeFolder("Prebuilt Branches", prebuilt_nodes),
      TreeFolder("Non-Prebuilt Branches", non_prebuilt_nodes),
    ]

    def _on_branch_selected(result):
      if result == DialogResult.CONFIRM and self._branch_dialog is not None:
        selection = self._branch_dialog.selection_ref
        if selection:
          ui_state.params.put("UpdaterTargetBranch", selection)
          self._branch_btn.action_item.set_value(selection)
          subprocess.run(["pkill", "-SIGUSR1", "-f", "openpilot.system.updated.updated"], check=False)
      self._branch_dialog = None

    self._branch_dialog = TreeOptionDialog(tr("Select a branch"), folders, current_target, "",
                                           on_exit=_on_branch_selected)

    gui_app.push_widget(self._branch_dialog)

  def _update_state(self):
    super()._update_state()
    show_advanced = ui_state.params.get_bool("ShowAdvancedControls")
    self.disable_updates_toggle.action_item.set_enabled(ui_state.is_offroad())
    self.disable_updates_toggle.set_visible(show_advanced)
    self.konik_toggle.action_item.set_enabled(ui_state.is_offroad() and not is_konik_locked(ui_state.params))
    self.konik_toggle.set_description(tr(DESCRIPTIONS["konik_locked"] if is_konik_locked(ui_state.params) else DESCRIPTIONS["konik_unlocked"]))

    disable_updates_desc = tr(DESCRIPTIONS["disable_updates_offroad"] if ui_state.is_offroad() else DESCRIPTIONS["disable_updates_onroad"])
    self.disable_updates_toggle.set_description(disable_updates_desc)
