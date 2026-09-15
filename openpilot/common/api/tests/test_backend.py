import os
from pathlib import Path

import pytest

import openpilot.common.api as connect_api
import openpilot.common.api.backend as backend
from openpilot.common.api import Api, api_get
from openpilot.common.api.backend import (
  COMMA_BACKEND,
  KONIK_BACKEND,
  active_dongle_id,
  backend_config,
  connect_client,
  enable_interlock,
  enforce_backend_state,
  is_konik_locked,
  lock_sentinel,
  pairing_url,
  set_konik_enabled,
  use_konik,
)


class FakeParams:
  def __init__(self, root: Path, values=None):
    self.values = values or {}
    self.param_path = root / "params" / "d"
    self.param_path.mkdir(parents=True)
    self.writes = []
    self.fail_on = None
    self.silent_fail_on = None

  def get_param_path(self):
    return str(self.param_path)

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key):
    return self.values.get(key)

  def put_bool(self, key, value, block=False):
    assert block
    self.writes.append((key, value))
    if key == self.fail_on:
      raise OSError("injected write failure")
    if key == self.silent_fail_on:
      return
    self.values[key] = value


@pytest.mark.parametrize(("values", "expected"), [
  ({}, "comma"),
  ({"UseKonikServer": True}, "konik"),
  ({"KonikLockout": True}, "konik"),
  ({"KonikInterlock": True}, "konik"),
])
def test_effective_backend_matrix(tmp_path, values, expected):
  assert backend_config(FakeParams(tmp_path, values)).name == expected


def test_standalone_konik_can_disable_only_before_lock(tmp_path):
  params = FakeParams(tmp_path)
  set_konik_enabled(params, True)
  set_konik_enabled(params, False)
  assert not use_konik(params)

  params.values["KonikLockout"] = True
  with pytest.raises(RuntimeError, match="factory reset"):
    set_konik_enabled(params, False)


def test_enable_interlock_locks_backend(tmp_path):
  params = FakeParams(tmp_path)
  enable_interlock(params)
  assert lock_sentinel(params).exists()
  assert params.values == {
    "KonikLockout": True,
    "UseKonikServer": True,
    "KonikInterlock": True,
  }


def test_partial_interlock_enable_fails_closed_and_recovers(tmp_path):
  params = FakeParams(tmp_path)
  params.fail_on = "KonikLockout"
  with pytest.raises(OSError, match="injected"):
    enable_interlock(params)
  assert lock_sentinel(params).exists()
  assert use_konik(params)

  params.fail_on = None
  enable_interlock(params)
  assert params.get_bool("KonikLockout")
  assert params.get_bool("UseKonikServer")
  assert params.get_bool("KonikInterlock")


@pytest.mark.parametrize("failed_key", [
  "KonikLockout",
  "UseKonikServer",
  "KonikInterlock",
])
def test_partial_interlock_enable_detects_silent_write_failure(tmp_path, failed_key):
  params = FakeParams(tmp_path)
  params.silent_fail_on = failed_key

  with pytest.raises(OSError, match=failed_key):
    enable_interlock(params)

  assert lock_sentinel(params).exists()
  assert use_konik(params)


def test_clearing_interlock_preserves_locked_backend(tmp_path):
  params = FakeParams(tmp_path, {
    "KonikLockout": True,
    "UseKonikServer": True,
    "KonikInterlock": True,
  })
  params.put_bool("KonikInterlock", False, block=True)
  enforce_backend_state(params)
  assert params.get_bool("KonikLockout")
  assert params.get_bool("UseKonikServer")
  assert use_konik(params)


def test_enforce_repairs_tampered_locked_state(tmp_path):
  params = FakeParams(tmp_path, {"KonikLockout": True, "UseKonikServer": False})
  enforce_backend_state(params)
  assert lock_sentinel(params).exists()
  assert params.get_bool("UseKonikServer")


def test_sentinel_is_durable_lock_source(tmp_path):
  params = FakeParams(tmp_path)
  lock_sentinel(params).touch()
  assert is_konik_locked(params)
  enforce_backend_state(params)
  assert params.get_bool("KonikLockout")
  assert params.get_bool("UseKonikServer")


def test_locked_backend_requires_konik_identity(tmp_path):
  params = FakeParams(tmp_path, {"KonikLockout": True})
  with pytest.raises(RuntimeError, match="KonikDongleId"):
    active_dongle_id(params)
  with pytest.raises(RuntimeError, match="KonikDongleId"):
    connect_client(params)
  config, client = connect_client(params, allow_unregistered=True)
  assert config == KONIK_BACKEND
  assert client.dongle_id is None


def test_connect_client_rejects_cached_unregistered_identity(tmp_path):
  params = FakeParams(tmp_path, {"UseKonikServer": True, "KonikDongleId": backend.UNREGISTERED_DONGLE_ID})
  with pytest.raises(RuntimeError, match="KonikDongleId"):
    connect_client(params)
  assert connect_client(params, allow_unregistered=True)[1].dongle_id == backend.UNREGISTERED_DONGLE_ID


def test_connect_client_returns_one_backend_snapshot(tmp_path, monkeypatch):
  params = FakeParams(tmp_path, {"CommaDongleId": "comma-id", "KonikDongleId": "konik-id"})
  selections = iter((COMMA_BACKEND, KONIK_BACKEND))
  monkeypatch.setattr(backend, "backend_config", lambda _: next(selections))
  config, client = connect_client(params)
  assert config == COMMA_BACKEND
  assert client.api_host == COMMA_BACKEND.api_host
  assert client.dongle_id == "comma-id"


def test_lock_sentinel_is_fsynced_with_parent(tmp_path, monkeypatch):
  params = FakeParams(tmp_path)
  sentinel = lock_sentinel(params)
  events = []
  fd_paths = {}
  real_open = os.open
  real_write = os.write
  real_fsync = os.fsync

  def tracked_open(path, flags, mode=0o777):
    fd = real_open(path, flags, mode)
    fd_paths[fd] = Path(path)
    events.append(("open", Path(path)))
    return fd

  def tracked_write(fd, data):
    events.append(("write", fd_paths[fd]))
    return real_write(fd, data)

  def tracked_fsync(fd):
    events.append(("fsync", fd_paths[fd]))
    return real_fsync(fd)

  monkeypatch.setattr(backend.os, "open", tracked_open)
  monkeypatch.setattr(backend.os, "write", tracked_write)
  monkeypatch.setattr(backend.os, "fsync", tracked_fsync)

  backend.create_lock_sentinel(params)
  assert sentinel.read_bytes() == b"1"
  assert [event for event in events if event[0] == "write"] == [("write", sentinel)]
  assert [event for event in events if event[0] == "fsync"] == [("fsync", sentinel), ("fsync", sentinel.parent)]

  events.clear()
  backend.create_lock_sentinel(params)
  assert not [event for event in events if event[0] == "write"]
  assert [event for event in events if event[0] == "fsync"] == [("fsync", sentinel), ("fsync", sentinel.parent)]


def test_backend_endpoints_and_pairing_urls(tmp_path):
  assert COMMA_BACKEND.api_host == "https://api.commadotai.com"
  assert COMMA_BACKEND.athena_host == "wss://athena.comma.ai"
  assert COMMA_BACKEND.pairing_host == "https://connect.comma.ai"
  assert COMMA_BACKEND.dongle_param == "CommaDongleId"
  assert KONIK_BACKEND.api_host == "https://api.konik.ai"
  assert KONIK_BACKEND.athena_host == "wss://athena.konik.ai"
  assert KONIK_BACKEND.pairing_host == "https://stable.konik.ai"
  assert KONIK_BACKEND.dongle_param == "KonikDongleId"

  params = FakeParams(tmp_path)
  assert pairing_url(params, "token") == "https://connect.comma.ai/?pair=token"
  set_konik_enabled(params, True)
  assert pairing_url(params, "token") == "https://stable.konik.ai/?pair=token"


def test_long_lived_api_resolves_backend_and_identity_per_operation(tmp_path, monkeypatch):
  params = FakeParams(tmp_path, {"CommaDongleId": "comma-id", "KonikDongleId": "konik-id"})
  calls = []

  class RecordingClient:
    def __init__(self, dongle_id, api_host):
      self.dongle_id = dongle_id
      self.api_host = api_host

    def request(self, method, endpoint, **kwargs):
      calls.append((self.dongle_id, self.api_host, method, endpoint))

  monkeypatch.setattr(connect_api, "Params", lambda: params)
  monkeypatch.setattr(backend, "CommaConnectApi", RecordingClient)
  api = Api("legacy-id")

  api.request("GET", "v1/test")
  params.values["KonikLockout"] = True
  api.request("GET", "v1/test")

  assert calls == [
    ("comma-id", COMMA_BACKEND.api_host, "GET", "v1/test"),
    ("konik-id", KONIK_BACKEND.api_host, "GET", "v1/test"),
  ]


def test_registration_api_get_routes_without_registered_identity(tmp_path, monkeypatch):
  params = FakeParams(tmp_path, {"UseKonikServer": True})
  calls = []

  class RecordingClient:
    def __init__(self, dongle_id, api_host):
      assert dongle_id is None
      self.api_host = api_host

    def api_get(self, endpoint, method, timeout, access_token, session, **kwargs):
      calls.append((self.api_host, endpoint, method))

  monkeypatch.setattr(connect_api, "Params", lambda: params)
  monkeypatch.setattr(backend, "CommaConnectApi", RecordingClient)
  api_get("v2/pilotauth/", method="POST")
  assert calls == [(KONIK_BACKEND.api_host, "v2/pilotauth/", "POST")]


def test_non_registration_api_get_requires_backend_identity(tmp_path, monkeypatch):
  params = FakeParams(tmp_path, {"UseKonikServer": True})
  monkeypatch.setattr(connect_api, "Params", lambda: params)

  with pytest.raises(RuntimeError, match="KonikDongleId"):
    api_get("v1/devices")
