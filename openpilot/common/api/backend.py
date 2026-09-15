from dataclasses import dataclass
import os
from pathlib import Path

from openpilot.common.api.comma_connect import API_HOST, CommaConnectApi
from openpilot.common.params import Params


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"


@dataclass(frozen=True)
class BackendConfig:
  name: str
  api_host: str
  athena_host: str
  pairing_host: str
  dongle_param: str


COMMA_BACKEND = BackendConfig(
  "comma",
  API_HOST,
  "wss://athena.comma.ai",
  "https://connect.comma.ai",
  "CommaDongleId",
)
KONIK_BACKEND = BackendConfig(
  "konik",
  "https://api.konik.ai",
  "wss://athena.konik.ai",
  "https://stable.konik.ai",
  "KonikDongleId",
)


def lock_sentinel(params: Params) -> Path:
  return Path(params.get_param_path()).parent / ".konik_lockout"


def create_lock_sentinel(params: Params) -> None:
  path = lock_sentinel(params)
  try:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
  except FileExistsError:
    fd = os.open(path, os.O_RDWR)

  try:
    if os.fstat(fd).st_size == 0:
      os.write(fd, b"1")
    os.fsync(fd)
  finally:
    os.close(fd)

  dir_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
  try:
    os.fsync(dir_fd)
  finally:
    os.close(dir_fd)


def is_konik_locked(params: Params) -> bool:
  return lock_sentinel(params).exists() or params.get_bool("KonikLockout") or params.get_bool("KonikInterlock")


def use_konik(params: Params) -> bool:
  return is_konik_locked(params) or params.get_bool("UseKonikServer")


def backend_config(params: Params) -> BackendConfig:
  return KONIK_BACKEND if use_konik(params) else COMMA_BACKEND


def put_bool_checked(params: Params, key: str, value: bool) -> None:
  params.put_bool(key, value, block=True)
  if params.get_bool(key) != value:
    raise OSError(f"failed to persist {key}")


def enforce_backend_state(params: Params) -> None:
  if is_konik_locked(params):
    create_lock_sentinel(params)
    put_bool_checked(params, "KonikLockout", True)
    put_bool_checked(params, "UseKonikServer", True)


def enable_interlock(params: Params) -> None:
  create_lock_sentinel(params)
  put_bool_checked(params, "KonikLockout", True)
  put_bool_checked(params, "UseKonikServer", True)
  put_bool_checked(params, "KonikInterlock", True)


def set_konik_enabled(params: Params, enabled: bool) -> None:
  if not enabled and is_konik_locked(params):
    raise RuntimeError("Konik cannot be disabled while locked; factory reset is required")
  put_bool_checked(params, "UseKonikServer", enabled)


def _active_dongle_id(params: Params, config: BackendConfig, allow_unregistered: bool) -> str | None:
  dongle_id = params.get(config.dongle_param)
  if dongle_id in (None, UNREGISTERED_DONGLE_ID) and not allow_unregistered:
    raise RuntimeError(f"{config.name} backend requires {config.dongle_param}")
  return dongle_id


def active_dongle_id(params: Params, allow_unregistered: bool = False) -> str | None:
  return _active_dongle_id(params, backend_config(params), allow_unregistered)


def connect_client(params: Params, allow_unregistered: bool = False) -> tuple[BackendConfig, CommaConnectApi]:
  config = backend_config(params)
  dongle_id = _active_dongle_id(params, config, allow_unregistered)
  return config, CommaConnectApi(dongle_id, config.api_host)


def pairing_url(params: Params, token: str) -> str:
  return f"{backend_config(params).pairing_host}/?pair={token}"
