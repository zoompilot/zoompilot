#!/usr/bin/env python3
import time
import json
import jwt
from typing import cast
from pathlib import Path

from datetime import datetime, timedelta, UTC
from openpilot.common.api import get_key_pair
from openpilot.common.api.backend import COMMA_BACKEND, UNREGISTERED_DONGLE_ID, backend_config, connect_client
from openpilot.common.params import Params
from openpilot.common.spinner import Spinner
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.common.hardware import HARDWARE, PC
from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog


def is_registered_device() -> bool:
  dongle = Params().get("DongleId")
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def _persisted_comma_id() -> str | None:
  path = Path(Paths.persist_root()) / "comma/dongle_id"
  if path.is_file():
    dongle_id = path.read_text().strip()
    return dongle_id or None
  return None


def _seed_comma_identity(params: Params) -> str | None:
  dongle_id = params.get("CommaDongleId")
  if dongle_id is None:
    legacy_id = params.get("DongleId")
    konik_id = params.get("KonikDongleId")
    dongle_id = legacy_id if legacy_id not in (None, "", UNREGISTERED_DONGLE_ID, konik_id) else _persisted_comma_id()
    if dongle_id is not None:
      params.put("CommaDongleId", dongle_id, block=True)
  return dongle_id


def register(show_spinner=False) -> str | None:
  """Register and mirror the identity for the effective Connect backend."""
  params = Params()
  while True:
    config, service = connect_client(params, allow_unregistered=True)
    dongle_id = _seed_comma_identity(params) if config == COMMA_BACKEND else params.get(config.dongle_param)

    # Create registration token, in the future, this key will make JWTs directly
    jwt_algo, private_key, public_key = get_key_pair()

    if not public_key:
      dongle_id = UNREGISTERED_DONGLE_ID
      cloudlog.warning("missing public key")
    elif dongle_id in (None, UNREGISTERED_DONGLE_ID):
      spinner = Spinner() if show_spinner else None
      if spinner is not None:
        spinner.update("registering device")

      serial = HARDWARE.get_serial()
      start_time = time.monotonic()
      imei: str | None = None
      while imei is None and backend_config(params) == config:
        try:
          imei = HARDWARE.get_imei()
        except Exception:
          cloudlog.exception("Error getting imei, trying again...")
          time.sleep(1)

        if time.monotonic() - start_time > 60 and spinner is not None:
          spinner.update(f"registering device - serial: {serial}, IMEI: {imei}")

      backoff = 0
      start_time = time.monotonic()
      while imei is not None and backend_config(params) == config:
        try:
          register_token = jwt.encode({'register': True, 'exp': datetime.now(UTC).replace(tzinfo=None) + timedelta(hours=1)},
                                      cast(str, private_key), algorithm=jwt_algo)
          cloudlog.info("getting pilotauth")
          resp = service.api_get("v2/pilotauth/", method='POST', timeout=15,
                                 imei=imei, imei2="", serial=serial, public_key=public_key, register_token=register_token)

          if resp.status_code in (402, 403):
            cloudlog.info(f"Unable to register device, got {resp.status_code}")
            dongle_id = UNREGISTERED_DONGLE_ID
          else:
            dongle_id = json.loads(resp.text)["dongle_id"]
          break
        except NotImplementedError:
          # dependency issues with PyJWT will hang the registration test in backoff loop otherwise
          raise
        except Exception:
          cloudlog.exception("failed to authenticate")
          backoff = min(backoff + 1, 15)
          time.sleep(backoff)

        if time.monotonic() - start_time > 60 and spinner is not None:
          spinner.update(f"registering device - serial: {serial}, IMEI: {imei}")
          dongle_id = UNREGISTERED_DONGLE_ID  # hotfix to prevent an infinite wait for registration
          break

      if spinner is not None:
        spinner.close()

    if backend_config(params) != config:
      continue
    if dongle_id:
      params.put(config.dongle_param, dongle_id, block=True)
      params.put("DongleId", dongle_id, block=True)
      set_offroad_alert("Offroad_UnregisteredHardware", (dongle_id == UNREGISTERED_DONGLE_ID) and not PC)
    return dongle_id


if __name__ == "__main__":
  print(register())
