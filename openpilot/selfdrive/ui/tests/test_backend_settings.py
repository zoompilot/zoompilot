from types import SimpleNamespace

import pytest

import openpilot.selfdrive.ui.lib.api_helpers as api_helpers
import openpilot.selfdrive.ui.mici.widgets.pairing_dialog as mici_pairing
import openpilot.selfdrive.ui.sunnypilot.layouts.settings.software as software
import openpilot.selfdrive.ui.widgets.pairing_dialog as tici_pairing
from openpilot.system.ui.widgets import DialogResult


PAIRING_DIALOGS = (
  pytest.param(tici_pairing, "params", "qr_texture", id="standard"),
  pytest.param(mici_pairing, "_params", "_qr_texture", id="mici"),
)


def test_authenticated_api_get_uses_one_backend_snapshot(monkeypatch):
  config = object()
  calls = []

  class Api:
    dongle_id = "snapshot-id"

    def get_token(self, *, expiry_hours):
      assert expiry_hours == api_helpers.TOKEN_EXPIRY_HOURS
      return "snapshot-token"

    def api_get(self, endpoint, **kwargs):
      calls.append((endpoint, kwargs))
      return "response"

  params = object()
  monkeypatch.setattr(api_helpers, "system_time_valid", lambda: True)
  monkeypatch.setattr(api_helpers, "connect_client", lambda actual: (config, Api()) if actual is params else pytest.fail())
  monkeypatch.setattr(api_helpers, "backend_config", lambda actual: config if actual is params else pytest.fail())

  actual_config, response = api_helpers.authenticated_api_get(params, "v1/devices/{dongle_id}", session="session")

  assert actual_config is config
  assert response == "response"
  assert calls == [("v1/devices/snapshot-id", {"access_token": "snapshot-token", "session": "session"})]


@pytest.mark.parametrize("flip_at", ("token", "request"))
def test_authenticated_api_get_rejects_backend_transition(monkeypatch, flip_at):
  configs = [object(), object()]
  state = {"config": configs[0]}
  requests = []

  class Api:
    dongle_id = "comma-id"

    def get_token(self, *, expiry_hours):
      if flip_at == "token":
        state["config"] = configs[1]
      return "comma-token"

    def api_get(self, endpoint, **kwargs):
      requests.append((endpoint, kwargs))
      state["config"] = configs[1]
      return "stale-response"

  monkeypatch.setattr(api_helpers, "system_time_valid", lambda: True)
  monkeypatch.setattr(api_helpers, "connect_client", lambda _: (configs[0], Api()))
  monkeypatch.setattr(api_helpers, "backend_config", lambda _: state["config"])

  with pytest.raises(RuntimeError, match="backend changed"):
    api_helpers.authenticated_api_get(object(), "v1/devices/{dongle_id}")

  assert requests == ([] if flip_at == "token" else [("v1/devices/comma-id", {"access_token": "comma-token"})])


def bare_pairing_dialog(module, params_attr, texture_attr):
  dialog = object.__new__(module.PairingDialog)
  setattr(dialog, params_attr, object())
  setattr(dialog, texture_attr, None)
  return dialog


@pytest.mark.parametrize(("module", "params_attr", "texture_attr"), PAIRING_DIALOGS)
def test_pairing_url_uses_connect_snapshot(module, params_attr, texture_attr, monkeypatch):
  dialog = bare_pairing_dialog(module, params_attr, texture_attr)
  params = getattr(dialog, params_attr)
  calls = []

  class Api:
    def get_token(self, payload):
      assert payload == {"pair": True}
      return "token"

  def connect_client(actual_params):
    calls.append(actual_params)
    return SimpleNamespace(pairing_host="https://snapshot.example"), Api()

  monkeypatch.setattr(module, "connect_client", connect_client)

  assert dialog._get_pairing_url() == "https://snapshot.example/?pair=token"
  assert calls == [params]


@pytest.mark.parametrize(("module", "params_attr", "texture_attr"), PAIRING_DIALOGS)
def test_pairing_identity_failure_has_no_fallback_url(module, params_attr, texture_attr, monkeypatch):
  dialog = bare_pairing_dialog(module, params_attr, texture_attr)
  fallback_calls = []

  def fail_connect(_):
    raise RuntimeError("missing identity")

  monkeypatch.setattr(module, "connect_client", fail_connect)
  monkeypatch.setattr(module, "backend_config", lambda _: fallback_calls.append(True), raising=False)

  assert dialog._get_pairing_url() is None
  assert fallback_calls == []


@pytest.mark.parametrize(("module", "params_attr", "texture_attr"), PAIRING_DIALOGS)
def test_pairing_empty_token_renders_error(module, params_attr, texture_attr, monkeypatch):
  dialog = bare_pairing_dialog(module, params_attr, texture_attr)
  setattr(dialog, texture_attr, SimpleNamespace(id=0))
  monkeypatch.setattr(module, "connect_client", lambda _: (SimpleNamespace(pairing_host="https://snapshot.example"),
                                                            SimpleNamespace(get_token=lambda _: "")))
  monkeypatch.setattr(module, "make_texture", lambda *_: pytest.fail("must not encode a failed pairing URL"))

  dialog._generate_qr_code()

  assert getattr(dialog, texture_attr) is None


class FakeParams:
  def __init__(self, fail_on=None, silent=False):
    self.values = {"KonikInterlock": False}
    self.writes = []
    self.fail_on = fail_on
    self.silent = silent

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value, block=False):
    assert block
    self.writes.append((key, value))
    if key == self.fail_on:
      if self.silent:
        return
      raise OSError(f"failed {key}")
    self.values[key] = value


class FakeDialog:
  def __init__(self, *_, callback=None, **__):
    self.callback = callback


def bare_software_settings(monkeypatch, params, offroad, locked=False):
  pushed = []
  toggle_states = []
  state = SimpleNamespace(params=params, is_offroad=lambda: offroad[0])
  monkeypatch.setattr(software, "ui_state", state)
  monkeypatch.setattr(software, "ConfirmDialog", FakeDialog)
  monkeypatch.setattr(software, "alert_dialog", lambda _: "alert")
  monkeypatch.setattr(software.gui_app, "push_widget", pushed.append)
  monkeypatch.setattr(software.cloudlog, "exception", lambda _: None)
  monkeypatch.setattr(software, "is_konik_locked", lambda _: locked)
  monkeypatch.setattr(software, "use_konik", lambda p: p.get_bool("UseKonikServer"))
  monkeypatch.setattr(software, "set_konik_enabled",
                      lambda p, enabled: software.put_bool_checked(p, "UseKonikServer", enabled))

  settings = object.__new__(software.SoftwareLayoutSP)
  settings.konik_toggle = SimpleNamespace(action_item=SimpleNamespace(set_state=toggle_states.append))
  return settings, pushed, toggle_states


@pytest.mark.parametrize(("result", "offroad"), (
  (DialogResult.CANCEL, True),
  (DialogResult.CONFIRM, False),
))
def test_backend_noncommitting_exits_restore_toggle(monkeypatch, result, offroad):
  params = FakeParams()
  settings, pushed, toggle_states = bare_software_settings(monkeypatch, params, [offroad])

  settings._on_konik_toggled(True)
  pushed[0].callback(result)

  assert params.writes == []
  assert toggle_states == [False]


def test_locked_backend_cannot_be_disabled(monkeypatch):
  params = FakeParams()
  params.values["UseKonikServer"] = True
  settings, pushed, toggle_states = bare_software_settings(monkeypatch, params, [True], locked=True)

  settings._on_konik_toggled(False)

  assert pushed == []
  assert params.writes == []
  assert toggle_states == [True]


@pytest.mark.parametrize("silent", (False, True))
def test_backend_reboot_failure_rolls_back_and_reports_error(monkeypatch, silent):
  params = FakeParams(fail_on="DoReboot", silent=silent)
  settings, pushed, toggle_states = bare_software_settings(monkeypatch, params, [True])

  settings._on_konik_toggled(True)
  pushed[0].callback(DialogResult.CONFIRM)

  assert params.writes == [("UseKonikServer", True), ("DoReboot", True), ("UseKonikServer", False)]
  assert not params.get_bool("UseKonikServer")
  assert pushed[-1] == "alert"
  assert toggle_states == [False]


def test_backend_success_stages_reboot(monkeypatch):
  params = FakeParams()
  settings, pushed, toggle_states = bare_software_settings(monkeypatch, params, [True])

  settings._on_konik_toggled(True)
  pushed[0].callback(DialogResult.CONFIRM)

  assert params.writes == [("UseKonikServer", True), ("DoReboot", True)]
  assert toggle_states == [True]
