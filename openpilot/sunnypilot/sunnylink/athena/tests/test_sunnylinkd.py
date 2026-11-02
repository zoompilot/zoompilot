"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import base64
import gzip
import json
from types import SimpleNamespace
from unittest import mock

from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL
from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
from openpilot.common.test import OpenpilotTestCase


class TestSunnylinkdMethods(OpenpilotTestCase):
  def setup_method(self):
    self.saved_params = []

    self.original_save = sunnylinkd.save_param_from_base64_encoded_string

    def mock_save_param(key, value, compression=False):
      self.saved_params.append((key, value, compression))

    sunnylinkd.save_param_from_base64_encoded_string = mock_save_param  # ty: ignore[invalid-assignment]

  def teardown_method(self):
    sunnylinkd.save_param_from_base64_encoded_string = self.original_save  # ty: ignore[invalid-assignment]

  def test_saveParams_blocked(self):
    blocked_params = {
      "GithubUsername": "attacker",
      "GithubSshKeys": "ssh-rsa attacker_key",
      "OnroadCycleRequested": "1",
      "AlphaLongitudinalEnabled": "1",
    }

    sunnylinkd.saveParams(blocked_params)

    assert len(self.saved_params) == 0

  def test_saveParams_allowed(self):
    allowed_params = {
      "SpeedLimitOffset": "5",
      "MyCustomParam": "123"
    }

    sunnylinkd.saveParams(allowed_params)

    # verify content
    assert len(self.saved_params) == 2
    keys_saved = [p[0] for p in self.saved_params]
    assert "SpeedLimitOffset" in keys_saved
    assert "MyCustomParam" in keys_saved

  def test_saveParams_mixed(self):
    mixed_params = {
      "GithubUsername": "attacker",
      "SpeedLimitOffset": "10"
    }

    sunnylinkd.saveParams(mixed_params)

    # should save allowed one
    assert len(self.saved_params) == 1
    assert self.saved_params[0][0] == "SpeedLimitOffset"
    assert self.saved_params[0][1] == "10"

  def test_getParamsMetadata_default_big_model(self):
    jetlink = SimpleNamespace(default_model="Cinque Terre V3 Model")
    for chestnut, expected in ((False, "Cinque Terre V3 Model"), (True, DEFAULT_BIG_MODEL)):
      with mock.patch.object(sunnylinkd, 'chestnut_present', return_value=chestnut), \
           mock.patch.object(jetlink_adapter, 'status', return_value=jetlink):
        schema = json.loads(gzip.decompress(base64.b64decode(sunnylinkd.getParamsMetadata())))
      assert schema["default_big_model"] == expected
