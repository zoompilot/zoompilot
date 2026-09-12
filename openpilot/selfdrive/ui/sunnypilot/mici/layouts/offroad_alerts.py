"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.layouts.offroad_alerts import MiciOffroadAlerts
from openpilot.selfdrive.ui.mici.layouts.settings.software import _split_description
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.software import ReleaseNotesPage

UPDATE_KEY = "UpdateAvailable"


class MiciOffroadAlertsSP(MiciOffroadAlerts):
  """Upstream's update alert reboots on a tap and sends users to comma's blog for the notes.

  Ours opens the notes of the release that is waiting, which carry the install slider.
  """

  def __init__(self):
    super().__init__()

    self._notes_page = ReleaseNotesPage()
    self._update_item = next(item for item in self.alert_items if item.alert_data.key == UPDATE_KEY)
    self._update_item.set_click_callback(lambda: self._notes_page.show_notes(True))

  def _refresh(self, pending_params: dict) -> int:
    active_count = super()._refresh(pending_params)

    alert = self._update_item.alert_data
    if alert.visible:
      desc = _split_description(pending_params.get("UpdaterNewDescription") or "")
      version = f"\nzoompilot {desc[0]}, {desc[3]}\n" if desc is not None else ""
      alert.text = f"Update ready{version}. Tap to read what's new."
      self._update_item.update_alert_data(alert)

    return active_count
