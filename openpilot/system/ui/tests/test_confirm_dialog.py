from openpilot.system.ui.widgets import DialogResult
import openpilot.system.ui.widgets.confirm_dialog as confirm_dialog
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog


def bare_dialog(callback):
  dialog = object.__new__(ConfirmDialog)
  dialog._children = []
  dialog._callback = callback
  dialog._result = None
  return dialog


def test_external_dismiss_reports_cancel_once():
  results = []
  dialog = bare_dialog(results.append)

  dialog.hide_event()
  dialog.hide_event()

  assert results == [DialogResult.CANCEL]


def test_confirm_is_not_replaced_by_hide(monkeypatch):
  results = []
  dialog = bare_dialog(results.append)
  monkeypatch.setattr(confirm_dialog.gui_app, "pop_widget", dialog.hide_event)

  dialog._confirm_button_callback()

  assert results == [DialogResult.CONFIRM]
