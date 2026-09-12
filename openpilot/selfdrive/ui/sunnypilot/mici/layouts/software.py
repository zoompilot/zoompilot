"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import threading

import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.settings.software import InstallUpdateButton, SoftwareLayoutMici, _split_description
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.html_render import LIST_INDENT_PX, ElementType, HtmlRenderer
from openpilot.system.ui.widgets.scroller import NavRawScrollPanel
from openpilot.system.ui.widgets.slider import LargerSlider

NOTES_FONT_SIZE = 28  # the renderer defaults to the tici screen, mici is 536x240
SLIDER_MARGIN = 24


def update_pending() -> bool:
  return ui_state.params.get_bool("UpdateAvailable")


def no_notes() -> str:
  return f"<h2>{tr('No release notes available.')}</h2>"


class ReleaseNotesPage(NavRawScrollPanel):
  """Scrolling page for the notes updated writes to params: the top CHANGELOG.md entry, as html."""

  def __init__(self):
    super().__init__()
    self._content = HtmlRenderer(text=no_notes(), text_size={ElementType.P: NOTES_FONT_SIZE})
    # installing is a reboot: offer it under the notes of the release waiting to be installed
    self._install = self._child(LargerSlider("slide to install", confirm_callback=self._on_install))
    self._install.set_visible(lambda: ui_state.is_offroad() and update_pending())
    self._install.set_enabled(lambda: self.enabled and not self.is_dismissing)

  def show_notes(self, pending: bool):
    """Open on the staged release's notes, or on the running release's."""
    notes = ui_state.params.get("UpdaterNewReleaseNotes" if pending else "UpdaterCurrentReleaseNotes")
    self._content.parse_html_content((notes or b"").decode("utf-8", "replace").strip() or no_notes())
    gui_app.push_widget(self)

  def _on_install(self):
    threading.Thread(target=lambda: ui_state.params.put_bool("DoReboot", True, block=True), daemon=True).start()

  def _render(self, rect: rl.Rectangle):
    # the renderer indents a nested list without narrowing its wrap width, so leave the room here
    width = rect.width - LIST_INDENT_PX
    content_height = self._content.get_total_height(int(width))

    installable = self._install.is_visible
    total_height = content_height + (SLIDER_MARGIN + self._install.rect.height if installable else 0)
    scroll_offset = round(self._scroll_panel.update(rect, total_height))

    # the renderer draws every line inside the rect it is handed, so hand it the screen, not the notes
    self._content.render(rl.Rectangle(rect.x, rect.y + scroll_offset, width, rect.height - scroll_offset))

    if installable:
      self._install.render(rl.Rectangle(rect.x + (rect.width - self._install.rect.width) / 2,
                                        rect.y + scroll_offset + content_height + SLIDER_MARGIN,
                                        self._install.rect.width, self._install.rect.height))


class ReleaseNotesButton(BigButton):
  def __init__(self):
    super().__init__("release notes", "", gui_app.texture("icons_mici/settings/device/info.png", 64, 64))
    self._page = ReleaseNotesPage()
    self.set_click_callback(lambda: self._page.show_notes(update_pending()))

  def _update_state(self):
    super()._update_state()

    desc = _split_description(ui_state.params.get("UpdaterNewDescription" if update_pending() else "UpdaterCurrentDescription") or "")
    value = desc[0] if desc is not None else ""
    if self.get_value() != value:
      self.set_value(value)


class SoftwareLayoutSP(SoftwareLayoutMici):
  def __init__(self):
    super().__init__()

    # add_widget wraps the touch callback, so append and then move it up behind "install now"
    self._scroller.add_widget(ReleaseNotesButton())
    items = self._scroller.items
    install = next(i for i, item in enumerate(items) if isinstance(item, InstallUpdateButton))
    items.insert(install + 1, items.pop())
