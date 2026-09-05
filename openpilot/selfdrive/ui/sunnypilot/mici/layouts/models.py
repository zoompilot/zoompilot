"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.cereal import custom
from openpilot.sunnypilot import accelerators
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialog
from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS, get_selected_bundle
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigMultiToggle
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.selfdrive.ui.sunnypilot.model_info import (active_source, big_model_progress, big_model_state,
                                                          bundles_for_source, carrying_model,
                                                          default_model_name, model_info, queued_name)
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets.scroller import NavScroller

# The user's say over the accelerator link. Absent means auto and the backend
# decides from what it finds; true and false force it. The backend reads the
# param, this panel only writes it, so the name lives here and never on screen.
LINK_PARAM = "JetlinkEnabled"
LINK_STATES = ("auto", "on", "off")


def read_link_state() -> str:
  """auto | on | off. Never raises: a params library older than the key would
  otherwise take the settings panel down, the guard accelerators.progress() has."""
  try:
    value = ui_state.params.get(LINK_PARAM)
  except Exception:
    return "auto"
  return "auto" if value is None else "on" if value else "off"


def write_link_state(state: str) -> None:
  try:
    if state == "auto":
      ui_state.params.remove(LINK_PARAM)
    else:
      ui_state.params.put_bool(LINK_PARAM, state == "on", block=True)
    ui_state.params.remove('ModelRunnerTypeCache')
  except Exception:
    pass  # the same unknown-key case as the read; nothing the panel can do about it


def link_toggle_meaningful() -> bool:
  """Whether to show the toggle at all. A plain device with no accelerator, no
  complaint and nothing set must not. ready() is on the list for the link whose
  engine is cached while the hardware is out of the car: on auto that is exactly
  when someone wants to turn it off."""
  return (accelerators.present() or accelerators.ready() or read_link_state() != "auto"
          or accelerators.unavailable_reason() is not None)


class AcceleratorLinkToggle(BigMultiToggle):
  """auto / on / off over one bool param, where absent means auto.

  BigMultiParamToggle stores an option index and cannot say "unset", which is
  the state every device should stay in until someone decides otherwise.
  """

  def __init__(self):
    super().__init__(tr("accelerator link"), [tr(state) for state in LINK_STATES], select_callback=self._store)
    self.refresh()

  def _store(self, label: str) -> None:
    if not ui_state.is_offroad():
      return
    write_link_state(LINK_STATES[self._options.index(label)])

  def refresh(self) -> None:
    label = self._options[LINK_STATES.index(read_link_state())]
    if label != self.value:
      self.set_value(label)


def _model_info() -> tuple[str, str, str]:
  """(active model, info header, info text) for the panel. Runner-matched: the
  active line names what actually drives, and a notable big-model state takes
  the info pair."""
  source, active_name, other_name = model_info()
  state = big_model_state()
  _, _, carry_display = carrying_model()
  if carry_display is None:
    big = get_selected_bundle(ui_state.params, "chestnut")
    carry_display = big.displayName if big else default_model_name("chestnut")
  active_text = (carry_display or active_name).lower()
  provisioning = big_model_progress()
  if provisioning is not None:
    stage, frac, msg = provisioning
    if stage == 'failed':
      return active_text, tr("big model"), tr("unavailable")
    # The message when there is one: "waiting for the jetson" is worth more to
    # a driver than "connect 0%", and a stage with nothing to measure should
    # not be given a percentage that only ever reads zero.
    detail = tr(msg) if msg else tr(stage)
    return active_text, tr("big model"), f"{detail} {frac * 100:.0f}%" if frac > 0 else detail
  if state == 'failed':
    return active_text, tr("big model"), tr("unavailable")
  if state == 'loading':
    return active_text, tr("big model"), tr("getting ready")
  header = tr("small model") if source == "chestnut" else tr("big model")
  return active_text, header, other_name.lower()


class CurrentModelInfo(Widget):
  def __init__(self):
    super().__init__()

    self.set_rect(rl.Rectangle(0, 0, 360, 180))

    header_color = rl.Color(255, 255, 255, int(255 * 0.9))
    subheader_color = rl.Color(255, 255, 255, int(255 * 0.9 * 0.65))
    max_width = int(self._rect.width - 20)
    active_text, info_header, info_text = _model_info()
    self.current_model_header = UnifiedLabel(tr("active model"), 48, max_width=max_width, text_color=header_color, font_weight=FontWeight.DISPLAY)
    self.current_model_text = UnifiedLabel(active_text, 32, max_width=max_width, text_color=subheader_color, font_weight=FontWeight.ROMAN, scroll=True)

    self.info_header = UnifiedLabel(info_header, 48, max_width=max_width, text_color=header_color, font_weight=FontWeight.DISPLAY)
    self.info_text = UnifiedLabel(info_text, 32, max_width=max_width, text_color=subheader_color, font_weight=FontWeight.ROMAN, scroll=True)

  def _render(self, _):
    self.current_model_header.set_position(self._rect.x + 20, self._rect.y - 10)
    self.current_model_header.render()

    self.current_model_text.set_position(self._rect.x + 20, self._rect.y + 68 - 25)
    self.current_model_text.render()

    self.info_header.set_position(self._rect.x + 20, self._rect.y + 114 - 30)
    self.info_header.render()

    self.info_text.set_position(self._rect.x + 20, self._rect.y + 161 - 25)
    self.info_text.render()

class ModelsLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self.focused_widget = None

    self.current_model_info = CurrentModelInfo()
    self._download_progress = "."
    self._download_frame = 0
    self._was_downloading = False
    self._selection_source: str | None = None

    self.select_model_btn = BigButton(tr("select model"))
    self.select_model_btn.set_click_callback(self._show_folders)

    self.cancel_download_btn = BigButton(tr("cancel download"))
    self.cancel_download_btn.set_click_callback(lambda: ui_state.params.remove("ModelManager_DownloadRef"))

    self.link_toggle = AcceleratorLinkToggle()
    self.link_toggle.set_visible(link_toggle_meaningful())

    self.main_items = [self.current_model_info, self.select_model_btn, self.cancel_download_btn, self.link_toggle]
    self._scroller.add_widgets(self.main_items)

  @property
  def model_manager(self):
    return ui_state.sm["modelManagerSP"]

  def _get_grouped_bundles(self, bundles, favorites = None):
    folders = {}
    for bundle in bundles:
      folder = next((override.value for override in bundle.overrides if override.key == "folder"), "")
      folders.setdefault(folder, []).append(bundle)

    if favorites:
      for fav_bundle in [bundle for bundle in bundles if bundle.ref in favorites]:
        folders.setdefault("favorites", []).append(fav_bundle)

    return folders

  def _push_selection_view(self, items):
    scroller = NavScroller()
    scroller._scroller.add_widgets(items)
    gui_app.push_widget(scroller)

  def _show_folders(self):
    self.focused_widget = self.select_model_btn

    hardware_btns = []
    active = active_source()
    for source, label in (("qcom", tr("small models")), ("chestnut", tr("big models"))):
      bundle = get_selected_bundle(ui_state.params, source)
      value = (bundle.internalName if bundle else default_model_name(source)).lower()
      if source == active:
        value += f" ({tr('active')})"
      btn = BigButton(label.lower(), value=value)
      btn.set_click_callback(lambda s=source: self._select_hardware(s))
      hardware_btns.append(btn)
    choices = accelerators.model_choices()
    if choices:
      selected = next((m['name'] for m in choices if m['selected']), '')
      btn = BigButton(tr('accelerator models'), value=selected.lower())
      btn.set_click_callback(self._select_accelerator)
      hardware_btns.append(btn)
    self._push_selection_view(hardware_btns)

  def _select_accelerator(self):
    buttons = []
    for choice in accelerators.model_choices():
      value = tr('cached on accelerator') if choice['cached'] else tr('build required')
      if choice['selected']:
        value += f" ({tr('selected')})"
      btn = BigButton(choice['name'].lower(), value=value)
      btn.set_click_callback(lambda m=choice: self._choose_accelerator(m))
      buttons.append(btn)
    self._push_selection_view(buttons)

  def _choose_accelerator(self, choice):
    if not ui_state.is_offroad():
      return
    accelerators.select_model(choice['backend'], choice['name'])
    self._pop_to_main()

  def _select_hardware(self, source):
    self._selection_source = source

    favs = ui_state.params.get("ModelManager_Favs")
    favorites = set(favs.split(';')) if favs else set()

    bundles = bundles_for_source(source)
    if not bundles:
      gui_app.push_widget(BigDialog(title=tr("No models available"),
                                    description=tr("No models are available for this hardware yet. Connect to the internet and refresh the model list.")))
      return
    folders = self._get_grouped_bundles(bundles, favorites)

    folder_buttons = []
    default_btn = BigButton(default_model_name(source).lower())
    default_btn.set_click_callback(lambda s=source: self._select_default(s))
    folder_buttons.append(default_btn)

    for folder in sorted(folders.keys(), key=lambda f: max((bundle.index for bundle in folders[f]), default=-1), reverse=True):
      btn = BigButton(folder.lower())
      btn.set_click_callback(lambda f=folder: self._select_folder(f))
      if folder.lower() == "favorites":
        folder_buttons.insert(0, btn)
      else:
        folder_buttons.append(btn)
    self._push_selection_view(folder_buttons)

  def _pop_to_main(self):
    gui_app.pop_widgets_to(self)
    self._scroller.scroll_panel.set_offset(0.0)

  def _select_model(self, bundle):
    ui_state.params.put("ModelManager_DownloadRef", bundle.ref)
    self._pop_to_main()

  def _select_default(self, source):
    ui_state.params.remove(ACTIVE_BUNDLE_KEYS[source])
    self._pop_to_main()

  def _select_folder(self, folder_name):
    source = self._selection_source
    if source is None:  # folders are only reachable after picking a hardware
      return
    favs = ui_state.params.get("ModelManager_Favs")
    favorites = set(favs.split(';')) if favs else set()

    folders = self._get_grouped_bundles(bundles_for_source(source), favorites)
    bundles = sorted(folders.get(folder_name, []), key=lambda b: b.index, reverse=True)

    btns = []
    for bundle in bundles:
      btn = BigButton(bundle.displayName.lower())
      btn.set_click_callback(lambda b=bundle: self._select_model(b))
      btns.append(btn)
    self._push_selection_view(btns)

  def hide_event(self):
    super().hide_event()
    if self._was_downloading:
      device.set_override_interactive_timeout(None)
      self._was_downloading = False

  def _update_state(self):
    super()._update_state()

    self.select_model_btn.set_enabled(ui_state.is_offroad())
    self.cancel_download_btn.set_visible(False)
    self.current_model_info.current_model_header._shimmer = False
    self.current_model_info.info_header._shimmer = False

    manager = self.model_manager
    self._download_frame += 1
    should_update = self._download_frame % (gui_app.target_fps / 2) == 0
    if should_update:
      self._download_progress = self._download_progress + "." if len(self._download_progress) < 3 else ""
      # present() and unavailable_reason() read sysfs, so they ride this half-second
      # tick rather than the frame
      self.link_toggle.refresh()
      self.link_toggle.set_visible(link_toggle_meaningful())

    is_downloading = (manager.selectedBundle
                      and manager.selectedBundle.status == custom.ModelManagerSP.DownloadStatus.downloading)
    if self._was_downloading and not is_downloading:
      device.set_override_interactive_timeout(None)
    self._was_downloading = is_downloading

    self.current_model_info.current_model_header.set_text(tr("active model"))
    active_text, info_header, info_text = _model_info()
    self.current_model_info.current_model_text.set_text(active_text)
    self.current_model_info.info_header.set_text(info_header)
    self.current_model_info.info_text.set_text(info_text)

    if manager.selectedBundle and manager.selectedBundle.status == custom.ModelManagerSP.DownloadStatus.failed:
      self.current_model_info.info_header.set_text(tr("error") + self._download_progress)
      self.current_model_info.info_text.set_text(tr("download failed"))

    elif manager.selectedBundle and manager.selectedBundle.status == custom.ModelManagerSP.DownloadStatus.downloading:
      self.cancel_download_btn.set_visible(True)
      device.set_override_interactive_timeout(5)
      progress = 0.0
      count = 0
      verifying = False
      for model in manager.selectedBundle.models:
        count += 1
        p = model.artifact.downloadProgress
        if p.status in (custom.ModelManagerSP.DownloadStatus.downloading,
                        custom.ModelManagerSP.DownloadStatus.verifying):
          progress += p.progress
          verifying = verifying or p.status == custom.ModelManagerSP.DownloadStatus.verifying
        elif p.status in (custom.ModelManagerSP.DownloadStatus.downloaded,
                          custom.ModelManagerSP.DownloadStatus.cached):
          progress += 100.0

      self.current_model_info.current_model_header.set_text(tr("verifying") if verifying else tr("downloading"))
      self.cancel_download_btn.set_text(tr("cancel verification") if verifying else tr("cancel download"))
      self.current_model_info.current_model_header._shimmer = True
      name_text = manager.selectedBundle.internalName.lower()
      if queued := queued_name(manager.selectedBundle.ref):
        name_text += f"  |  {queued.lower()} {tr('queued')}"
      self.current_model_info.current_model_text.set_text(name_text)
      self.current_model_info.info_header.set_text(tr("progress") + self._download_progress)
      self.current_model_info.info_header._shimmer = True
      self.current_model_info.info_text.set_text(f"{progress/count:.2f}%")

    elif manager.selectedBundle and manager.selectedBundle.status == custom.ModelManagerSP.DownloadStatus.downloaded:
      self.current_model_info.info_header.set_text(tr("downloaded"))
      self.current_model_info.info_text.set_text(tr("downloaded"))
