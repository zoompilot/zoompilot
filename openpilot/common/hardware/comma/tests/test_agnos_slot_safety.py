import json
import subprocess
from types import SimpleNamespace

import pytest

from openpilot.common.hardware.comma import agnos


def _manifest(tmp_path, partitions=()):
  path = tmp_path / "agnos.json"
  path.write_text(json.dumps(list(partitions)))
  return str(path)


def _cloudlog():
  return SimpleNamespace(info=lambda *_: None, error=lambda *_: None, exception=lambda *_: None)


@pytest.mark.parametrize(("active_slot", "expected_target"), [("_a", 1), ("_b", 0)])
def test_target_slot_is_always_inactive(monkeypatch, active_slot, expected_target):
  monkeypatch.setattr(agnos.subprocess, "check_output", lambda *_args, **_kwargs: active_slot)
  assert agnos.get_target_slot_number() == expected_target


@pytest.mark.parametrize("active_slot", ["", "_c", "_a\n_b"])
def test_active_slot_rejects_ambiguous_abctl_output(monkeypatch, active_slot):
  monkeypatch.setattr(agnos.subprocess, "check_output", lambda *_args, **_kwargs: active_slot)
  with pytest.raises(RuntimeError, match="unexpected active slot"):
    agnos.get_active_slot_number()


def test_slot_suffix_rejects_invalid_slot_number():
  with pytest.raises(ValueError, match="invalid slot number"):
    agnos.slot_number_to_suffix(2)


def test_flash_aborts_if_target_cannot_be_marked_unbootable(tmp_path, monkeypatch):
  def fail(*_args, **_kwargs):
    raise subprocess.CalledProcessError(1, "abctl")

  monkeypatch.setattr(agnos, "get_active_slot_number", lambda: 0)
  monkeypatch.setattr(agnos.subprocess, "run", fail)

  with pytest.raises(subprocess.CalledProcessError):
    agnos.flash_agnos_update(_manifest(tmp_path, [{"name": "boot"}]), 1, _cloudlog())


def test_empty_manifest_never_verifies_or_mutates_slots(tmp_path, monkeypatch):
  manifest = _manifest(tmp_path)
  calls = []
  monkeypatch.setattr(agnos, "get_active_slot_number", lambda: 0)
  monkeypatch.setattr(agnos.subprocess, "run", lambda *args, **kwargs: calls.append((args, kwargs)))
  monkeypatch.setattr(agnos, "activate_slot", lambda *_: pytest.fail("must not activate an empty manifest"))

  assert not agnos.verify_agnos_update(manifest, 1)
  monkeypatch.setattr(agnos, "get_target_slot_number", lambda: 1)
  assert agnos.main(["--verify-only", manifest]) == 1
  with pytest.raises(RuntimeError, match="verification failed"):
    agnos.swap(manifest, 1, _cloudlog())
  with pytest.raises(ValueError, match="must contain"):
    agnos.flash_agnos_update(manifest, 1, _cloudlog())

  assert calls == []


def test_manifest_must_be_a_list(tmp_path):
  manifest = tmp_path / "agnos.json"
  manifest.write_text(json.dumps({"name": "boot"}))

  with pytest.raises(ValueError, match="must be a list"):
    agnos.load_manifest(str(manifest))


def test_activation_retries_then_requires_postcondition(monkeypatch):
  active_slots = iter((0, 0, 1))
  calls, delays = [], []
  monkeypatch.setattr(agnos, "get_active_slot_number", lambda: next(active_slots))
  monkeypatch.setattr(agnos.subprocess, "run", lambda args, **kwargs: calls.append((args, kwargs)))
  monkeypatch.setattr(agnos.time, "sleep", delays.append)

  agnos.activate_slot(1, _cloudlog())

  assert calls == [
    (["abctl", "--set_active", "1"], {"check": True, "capture_output": True, "text": True}),
    (["abctl", "--set_active", "1"], {"check": True, "capture_output": True, "text": True}),
  ]
  assert delays == [agnos.ACTIVATION_RETRY_DELAY]


def test_activation_command_failure_is_bounded(monkeypatch):
  def fail(args, **kwargs):
    calls.append((args, kwargs))
    raise subprocess.CalledProcessError(1, args)

  monkeypatch.setattr(agnos, "get_active_slot_number", lambda: 0)
  calls, delays = [], []
  monkeypatch.setattr(agnos.subprocess, "run", fail)
  monkeypatch.setattr(agnos.time, "sleep", delays.append)

  with pytest.raises(RuntimeError, match="failed to activate"):
    agnos.activate_slot(1, _cloudlog())

  assert len(calls) == agnos.ACTIVATION_ATTEMPTS
  assert delays == [agnos.ACTIVATION_RETRY_DELAY] * (agnos.ACTIVATION_ATTEMPTS - 1)


def test_swap_verifies_before_activation(monkeypatch):
  monkeypatch.setattr(agnos, "get_active_slot_number", lambda: 0)
  monkeypatch.setattr(agnos, "verify_agnos_update", lambda *_: False)
  monkeypatch.setattr(agnos, "activate_slot", lambda *_: pytest.fail("must not activate an unverified slot"))

  with pytest.raises(RuntimeError, match="verification failed"):
    agnos.swap("manifest", 1, _cloudlog())


def test_verify_only_never_mutates_slots(monkeypatch):
  monkeypatch.setattr(agnos, "get_target_slot_number", lambda: 1)
  monkeypatch.setattr(agnos, "verify_agnos_update", lambda *_: True)
  monkeypatch.setattr(agnos, "swap", lambda *_: pytest.fail("verify-only must not swap"))
  monkeypatch.setattr(agnos, "flash_agnos_update", lambda *_args, **_kwargs: pytest.fail("verify-only must not flash"))

  assert agnos.main(["--verify-only", "manifest"]) == 0


def test_flash_verifies_each_partition_before_return(tmp_path, monkeypatch):
  manifest = _manifest(tmp_path, [{"name": "boot"}])
  monkeypatch.setattr(agnos, "get_active_slot_number", lambda: 0)
  monkeypatch.setattr(agnos.subprocess, "run", lambda *_args, **_kwargs: None)
  monkeypatch.setattr(agnos, "flash_partition", lambda *_args, **_kwargs: None)
  monkeypatch.setattr(agnos, "verify_partition", lambda *_args, **_kwargs: False)

  with pytest.raises(RuntimeError, match="verification failed for boot"):
    agnos.flash_agnos_update(manifest, 1, _cloudlog())


def test_swap_clears_fast_check_trailers_after_verify(tmp_path, monkeypatch):
  manifest = _manifest(tmp_path, [{"name": "system"}, {"name": "boot", "full_check": True}])
  order = []
  monkeypatch.setattr(agnos, "get_active_slot_number", lambda: 0)
  monkeypatch.setattr(agnos, "verify_agnos_update", lambda *_: order.append("verify") or True)
  monkeypatch.setattr(agnos, "clear_partition_hash", lambda _slot, p: order.append(f"clear:{p['name']}"))
  monkeypatch.setattr(agnos, "activate_slot", lambda *_: order.append("activate"))

  agnos.swap(manifest, 1, _cloudlog())

  assert order == ["verify", "clear:system", "activate"]
