import os

from openpilot.common.api.base import BaseApi

API_HOST = os.getenv('API_HOST', 'https://api.commadotai.com')


class CommaConnectApi(BaseApi):
  def __init__(self, dongle_id, api_host=None):
    super().__init__(dongle_id, api_host or API_HOST)
    self.user_agent = "openpilot-"
