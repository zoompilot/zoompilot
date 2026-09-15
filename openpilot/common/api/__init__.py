from openpilot.common.api.backend import connect_client
from openpilot.common.api.comma_connect import CommaConnectApi
from openpilot.common.params import Params


class Api:
  def __init__(self, dongle_id):
    self.params = Params()

  def _service(self):
    return connect_client(self.params)[1]

  def request(self, method, endpoint, **params):
    return self._service().request(method, endpoint, **params)

  def get(self, *args, **kwargs):
    return self._service().get(*args, **kwargs)

  def post(self, *args, **kwargs):
    return self._service().post(*args, **kwargs)

  def get_token(self, payload_extra=None, expiry_hours=1):
    return self._service().get_token(payload_extra, expiry_hours)


def api_get(endpoint, method='GET', timeout=None, access_token=None, session=None, **params):
  service = connect_client(Params(), allow_unregistered=(endpoint == "v2/pilotauth/"))[1]
  return service.api_get(endpoint, method, timeout, access_token, session, **params)


def get_key_pair() -> tuple[str, str, str] | tuple[None, None, None]:
  return CommaConnectApi(None).get_key_pair()
