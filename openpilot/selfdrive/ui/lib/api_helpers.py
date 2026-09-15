from requests import Response

from openpilot.common.api.backend import BackendConfig, backend_config, connect_client
from openpilot.common.params import Params
from openpilot.common.time_helpers import system_time_valid

TOKEN_EXPIRY_HOURS = 2


def authenticated_api_get(params: Params, endpoint: str, **kwargs) -> tuple[BackendConfig, Response]:
  if not system_time_valid():
    raise RuntimeError("System time is not valid, cannot generate token")

  config, api = connect_client(params)
  token = api.get_token(expiry_hours=TOKEN_EXPIRY_HOURS)
  if backend_config(params) != config:
    raise RuntimeError("Connect backend changed before API request")

  response = api.api_get(endpoint.format(dongle_id=api.dongle_id), access_token=token, **kwargs)
  if backend_config(params) != config:
    raise RuntimeError("Connect backend changed during API request")
  return config, response
