import json
from Crypto.PublicKey import RSA
from pathlib import Path

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.api.comma_connect import CommaConnectApi
from openpilot.common.params import Params
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.system.athena.tests.helpers import MockResponse
from openpilot.common.hardware.hw import Paths


class TestRegistration(OpenpilotTestCase):

  def setup_method(self):
    self.params = Params()
    for key in ("DongleId", "CommaDongleId", "KonikDongleId", "UseKonikServer", "KonikLockout", "KonikInterlock"):
      self.params.remove(key)

    persist_dir = Path(Paths.persist_root()) / "comma"
    persist_dir.mkdir(parents=True, exist_ok=True)
    self.priv_key = persist_dir / "id_rsa"
    self.pub_key = persist_dir / "id_rsa.pub"
    self.dongle_id = persist_dir / "dongle_id"
    for path in (self.priv_key, self.pub_key, self.dongle_id):
      path.unlink(missing_ok=True)

  def _generate_keys(self):
    k = RSA.generate(2048)
    self.priv_key.write_bytes(k.export_key())
    self.pub_key.write_bytes(k.publickey().export_key())

  def _mock_registration(self, mocker, dongle_id="NEW_ID", status=200):
    self._generate_keys()
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_serial", return_value="serial")
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_imei", return_value="imei")
    response = MockResponse(json.dumps({"dongle_id": dongle_id}) if status == 200 else None, status)
    return mocker.patch.object(CommaConnectApi, "api_get", autospec=True, return_value=response)

  def test_first_comma_registration(self, mocker):
    api_get = self._mock_registration(mocker, "comma-new")

    assert register() == "comma-new"
    assert api_get.call_count == 1
    assert api_get.call_args.args[0].api_host == "https://api.commadotai.com"
    assert self.params.get("CommaDongleId") == "comma-new"
    assert self.params.get("DongleId") == "comma-new"
    assert self.params.get("KonikDongleId") is None

  def test_first_konik_registration_does_not_consume_comma_identity(self, mocker):
    self.params.put_bool("UseKonikServer", True, block=True)
    self.params.put("DongleId", "legacy-or-tampered", block=True)
    self.dongle_id.write_text("persist-comma")
    api_get = self._mock_registration(mocker, "konik-new")

    assert register() == "konik-new"
    assert api_get.call_args.args[0].api_host == "https://api.konik.ai"
    assert self.params.get("KonikDongleId") == "konik-new"
    assert self.params.get("DongleId") == "konik-new"
    assert self.params.get("CommaDongleId") is None

    self.params.put_bool("UseKonikServer", False, block=True)
    assert register() == "persist-comma"
    assert self.params.get("CommaDongleId") == "persist-comma"
    assert self.params.get("KonikDongleId") == "konik-new"

  def test_reversible_prelock_switch_restores_each_identity(self, mocker):
    self._generate_keys()
    self.params.put("CommaDongleId", "comma-id", block=True)
    self.params.put("KonikDongleId", "konik-id", block=True)
    api_get = mocker.patch.object(CommaConnectApi, "api_get", autospec=True)

    self.params.put_bool("UseKonikServer", True, block=True)
    assert register() == "konik-id"
    assert self.params.get("DongleId") == "konik-id"

    self.params.put_bool("UseKonikServer", False, block=True)
    assert register() == "comma-id"
    assert self.params.get("DongleId") == "comma-id"
    assert not api_get.called

  def test_locked_konik_ignores_tampered_active_and_persisted_comma(self, mocker):
    self._generate_keys()
    self.params.put_bool("KonikLockout", True, block=True)
    self.params.put("KonikDongleId", "konik-id", block=True)
    self.params.put("DongleId", "tampered", block=True)
    self.dongle_id.write_text("persist-comma")
    api_get = mocker.patch.object(CommaConnectApi, "api_get", autospec=True)

    assert register() == "konik-id"
    assert self.params.get("DongleId") == "konik-id"
    assert self.params.get("CommaDongleId") is None
    assert not api_get.called

  def test_locked_missing_konik_registers_konik_not_tampered_active(self, mocker):
    self.params.put_bool("KonikLockout", True, block=True)
    self.params.put("DongleId", "tampered", block=True)
    api_get = self._mock_registration(mocker, "konik-new")

    assert register() == "konik-new"
    assert api_get.call_args.args[0].api_host == "https://api.konik.ai"
    assert self.params.get("KonikDongleId") == "konik-new"
    assert self.params.get("CommaDongleId") is None

  def test_comma_identity_migration_precedence(self, mocker):
    self._generate_keys()
    api_get = mocker.patch.object(CommaConnectApi, "api_get", autospec=True)
    self.params.put("CommaDongleId", "comma-param", block=True)
    self.params.put("DongleId", "legacy-active", block=True)
    self.dongle_id.write_text("persist-comma")

    assert register() == "comma-param"
    assert self.params.get("DongleId") == "comma-param"
    assert not api_get.called

  def test_param_only_legacy_comma_identity_migrates_once(self, mocker):
    self._generate_keys()
    self.params.put("DongleId", "legacy-comma", block=True)
    api_get = mocker.patch.object(CommaConnectApi, "api_get", autospec=True)

    assert register() == "legacy-comma"
    assert self.params.get("CommaDongleId") == "legacy-comma"
    assert not api_get.called

  def test_persist_only_comma_identity_recovers_after_userdata_reset(self, mocker):
    self._generate_keys()
    self.dongle_id.write_text("persist-comma")
    api_get = mocker.patch.object(CommaConnectApi, "api_get", autospec=True)

    assert register() == "persist-comma"
    assert self.params.get("CommaDongleId") == "persist-comma"
    assert self.params.get("DongleId") == "persist-comma"
    assert not api_get.called

  def test_no_keys(self, mocker):
    api_get = mocker.patch.object(CommaConnectApi, "api_get", autospec=True)

    assert register() == UNREGISTERED_DONGLE_ID
    assert not api_get.called
    assert self.params.get("CommaDongleId") == UNREGISTERED_DONGLE_ID
    assert self.params.get("DongleId") == UNREGISTERED_DONGLE_ID

  def test_unregistered_response_is_cached_per_backend(self, mocker):
    api_get = self._mock_registration(mocker, status=402)

    assert register() == UNREGISTERED_DONGLE_ID
    assert api_get.call_count == 1
    assert self.params.get("CommaDongleId") == UNREGISTERED_DONGLE_ID
    assert self.params.get("DongleId") == UNREGISTERED_DONGLE_ID

  def test_backend_change_during_registration_restarts_with_fresh_snapshot(self, mocker):
    self._generate_keys()
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_serial", return_value="serial")
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_imei", return_value="imei")

    def respond(service, *args, **kwargs):
      if service.api_host == "https://api.commadotai.com":
        self.params.put_bool("UseKonikServer", True, block=True)
        return MockResponse(json.dumps({"dongle_id": "stale-comma"}), 200)
      return MockResponse(json.dumps({"dongle_id": "konik-new"}), 200)

    api_get = mocker.patch.object(CommaConnectApi, "api_get", autospec=True, side_effect=respond)
    assert register() == "konik-new"
    assert [call.args[0].api_host for call in api_get.call_args_list] == ["https://api.commadotai.com", "https://api.konik.ai"]
    assert self.params.get("CommaDongleId") is None
    assert self.params.get("KonikDongleId") == "konik-new"
    assert self.params.get("DongleId") == "konik-new"

  def test_timeout_rechecks_backend_before_caching_unregistered(self, mocker):
    self._generate_keys()
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_serial", return_value="serial")
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_imei", return_value="imei")
    mocker.patch("openpilot.system.athena.registration.time.sleep")
    mocker.patch("openpilot.system.athena.registration.time.monotonic", side_effect=[0, 0, 0, 61, 61, 61, 61])
    spinner = mocker.patch("openpilot.system.athena.registration.Spinner").return_value

    def respond(service, *args, **kwargs):
      if service.api_host == "https://api.commadotai.com":
        self.params.put_bool("UseKonikServer", True, block=True)
        raise OSError("backend changed during timeout")
      return MockResponse(json.dumps({"dongle_id": "konik-new"}), 200)

    api_get = mocker.patch.object(CommaConnectApi, "api_get", autospec=True, side_effect=respond)
    assert register(show_spinner=True) == "konik-new"
    assert [call.args[0].api_host for call in api_get.call_args_list] == ["https://api.commadotai.com", "https://api.konik.ai"]
    assert spinner.close.call_count == 2
    assert self.params.get("CommaDongleId") is None
    assert self.params.get("KonikDongleId") == "konik-new"
    assert self.params.get("DongleId") == "konik-new"
