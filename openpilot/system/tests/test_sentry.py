from pathlib import Path
from types import SimpleNamespace

import pytest

import openpilot.system.sentry as sentry
import openpilot.system.tombstoned as tombstoned


def _build_metadata():
  return SimpleNamespace(
    channel_type="test",
    channel="branch",
    openpilot=SimpleNamespace(is_dirty=False, git_origin="origin", git_commit="commit"),
  )


@pytest.mark.parametrize("policy", [
  {"UseKonikServer": True},
  {"KonikLockout": True},
  {"KonikInterlock": True},
])
def test_init_is_blocked_before_any_sdk_call(mocker, tmp_path, policy):
  params = SimpleNamespace(
    get_bool=lambda key: policy.get(key, False),
    get_param_path=lambda: str(tmp_path / "params" / "d"),
  )
  sdk = mocker.patch.object(sentry, "sentry_sdk")
  mocker.patch.object(sentry, "Params", return_value=params)
  metadata = mocker.patch.object(sentry, "get_build_metadata")

  assert sentry.init(sentry.SentryProject.SELFDRIVE) is False
  assert not sdk.mock_calls
  assert not metadata.called


def test_comma_mode_emits(mocker):
  sdk = mocker.patch.object(sentry, "sentry_sdk")
  mocker.patch.object(sentry, "use_konik", return_value=False)
  mocker.patch.object(sentry, "get_build_metadata", return_value=_build_metadata())
  mocker.patch.object(sentry, "get_properties", return_value=("dongle", "user", "sunny"))
  mocker.patch.object(sentry, "get_version", return_value="version")
  mocker.patch.object(sentry.HARDWARE, "get_device_type", return_value="device")

  assert sentry.init(sentry.SentryProject.SELFDRIVE) is True
  sentry.set_tag("key", "value")
  sentry.set_user()
  sentry.capture_fingerprint("car", "name")

  sdk.init.assert_called_once()
  assert sdk.set_tag.called
  assert sdk.set_user.called
  sdk.capture_message.assert_called_once()
  sdk.flush.assert_called_once()


def test_live_konik_switch_blocks_every_sdk_path_but_keeps_local_crash(mocker, tmp_path):
  state = {"konik": False}
  sdk = mocker.patch.object(sentry, "sentry_sdk")
  mocker.patch.object(sentry, "use_konik", side_effect=lambda _: state["konik"])
  mocker.patch.object(sentry, "get_build_metadata", return_value=_build_metadata())
  mocker.patch.object(sentry, "get_properties", return_value=("dongle", "user", "sunny"))
  mocker.patch.object(sentry, "get_version", return_value="version")
  mocker.patch.object(sentry.HARDWARE, "get_device_type", return_value="device")
  mocker.patch.object(sentry, "CRASHES_DIR", str(tmp_path))

  assert sentry.init(sentry.SentryProject.SELFDRIVE) is True
  sdk.reset_mock()
  state["konik"] = True

  sentry.set_tag("key", "value")
  sentry.set_user()
  sentry.capture_fingerprint("car", "name")
  sentry.capture_fingerprint_mock()
  sentry.report_tombstone("fn", "message", "contents")
  sentry.capture_exception()
  assert sentry.init(sentry.SentryProject.SELFDRIVE) is False

  assert not sdk.mock_calls
  assert (Path(tmp_path) / "error.log").is_file()
  assert len(list(Path(tmp_path).glob("*.log"))) >= 2


def test_switch_during_helper_blocks_remaining_sdk_calls(mocker):
  sdk = mocker.patch.object(sentry, "sentry_sdk")
  mocker.patch.object(sentry, "use_konik", side_effect=[False, True, True])

  sentry.capture_fingerprint_mock()

  assert not sdk.mock_calls


def test_before_send_fails_closed_after_live_switch(mocker):
  state = {"konik": False}
  mocker.patch.object(sentry, "use_konik", side_effect=lambda _: state["konik"])
  event = {"message": "crash"}

  assert sentry._before_send(event, None) is event
  state["konik"] = True
  assert sentry._before_send(event, None) is None


def test_locked_native_tombstone_still_uses_local_archive_path(mocker, tmp_path):
  tombstone = tmp_path / "native.crash"
  tombstone.write_text("crash")
  mocker.patch.object(tombstoned.sentry, "init", return_value=False)
  mocker.patch.object(tombstoned, "clear_apport_folder")
  mocker.patch.object(tombstoned, "get_tombstones", side_effect=[[], [(str(tombstone), 1)]])
  archive = mocker.patch.object(tombstoned, "report_tombstone_apport")
  mocker.patch.object(tombstoned.time, "sleep", side_effect=RuntimeError("stop"))

  with pytest.raises(RuntimeError, match="stop"):
    tombstoned.main()

  archive.assert_called_once_with(str(tombstone))
