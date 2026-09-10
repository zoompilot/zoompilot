"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import multiprocessing
import os
import queue
import threading

from openpilot.system.athena import athenad
from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import ParamKeyFlag, Params


CONNECT_PROTECTED_PARAMS = {
  "UseKonikServer",
  "KonikLockout",
  "KonikInterlock",
  "CommaDongleId",
  "KonikDongleId",
  "DongleId",
  "AthenadUploadQueue",
  "SunnylinkUploadQueue",
}


def _cache_upload(param_key, source_backend):
  upload_queue = queue.PriorityQueue()
  athenad.UploadQueueCache.configure(param_key)
  upload_queue.put_nowait(athenad.UploadItem("_", "_", {}, 0, source_backend, source_backend=source_backend))
  athenad.UploadQueueCache.cache(upload_queue)


class TestSunnylinkdMethods(OpenpilotTestCase):
  def setup_method(self):
    self.saved_params = []
    self.cache_param_key = athenad.UploadQueueCache._param_key
    athenad.UploadQueueCache._param_key = None

    self.original_save = sunnylinkd.save_param_from_base64_encoded_string

    def mock_save_param(key, value, compression=False):
      self.saved_params.append((key, value, compression))

    sunnylinkd.save_param_from_base64_encoded_string = mock_save_param  # ty: ignore[invalid-assignment]

  def teardown_method(self):
    sunnylinkd.save_param_from_base64_encoded_string = self.original_save  # ty: ignore[invalid-assignment]
    athenad.UploadQueueCache._param_key = self.cache_param_key

  def test_saveParams_blocked(self):
    for value in ("attacker", "", "0"):
      sunnylinkd.saveParams(dict.fromkeys(CONNECT_PROTECTED_PARAMS | {"GithubUsername", "GithubSshKeys"}, value))

    assert len(self.saved_params) == 0

  def test_connect_security_params_are_not_remotely_readable(self):
    assert CONNECT_PROTECTED_PARAMS.isdisjoint(sunnylinkd.dispatcher["getParamsAllKeys"]())
    response = sunnylinkd.dispatcher["getParams"](sorted(CONNECT_PROTECTED_PARAMS))
    assert response == {"params": "[]"}

  def test_no_remote_param_remove_method(self):
    assert not {name for name in sunnylinkd.dispatcher if "param" in name.lower() and ("remove" in name.lower() or "delete" in name.lower())}

  def test_connect_security_params_are_excluded_from_backup(self):
    backup_keys = {key.decode() for key in Params().all_keys(ParamKeyFlag.BACKUP)}
    assert CONNECT_PROTECTED_PARAMS.isdisjoint(backup_keys)

  def test_upload_rpc_uses_sunnylink_provenance(self):
    athenad.upload_queue = queue.PriorityQueue()
    athenad.UploadQueueCache.configure("SunnylinkUploadQueue")
    os.makedirs(Paths.log_root(), exist_ok=True)
    with open(os.path.join(Paths.log_root(), "qlog.zst"), "wb"):
      pass

    end_event = threading.Event()
    thread = threading.Thread(target=sunnylinkd.jsonrpc_handler, args=(end_event, None, "sunnylink"))
    thread.start()
    try:
      params = {"fn": "qlog.zst", "url": "https://upload.example/qlog.zst", "headers": {}}
      athenad.recv_queue.put_nowait(json.dumps({"method": "uploadFileToUrl", "params": params, "jsonrpc": "2.0", "id": 1}))
      _, _, response, _ = athenad.send_queue.get(timeout=3)
      assert json.loads(response)["result"]["items"][0]["source_backend"] == "sunnylink"
      assert Params().get("SunnylinkUploadQueue")[0]["source_backend"] == "sunnylink"
    finally:
      end_event.set()
      thread.join()

  def test_local_proxy_uses_bounded_connection_and_read_timeouts(self, mocker):
    api = mocker.patch.object(sunnylinkd, "SunnylinkApi").return_value
    api.get_token.return_value = "jwt"
    ws = mocker.patch.object(sunnylinkd, "create_connection").return_value
    start_proxy = mocker.patch.object(sunnylinkd, "start_local_proxy_shim", return_value={"success": 1})
    end_event = threading.Event()

    assert sunnylinkd.startLocalProxy(end_event, "wss://proxy.example", 8022) == {"success": 1}
    assert sunnylinkd.create_connection.call_args.kwargs["timeout"] == athenad.LOCAL_PROXY_CONNECT_TIMEOUT
    ws.settimeout.assert_called_once_with(athenad.LOCAL_PROXY_READ_TIMEOUT)
    start_proxy.assert_called_once_with(end_event, 8022, ws)

  def test_connect_and_sunnylink_upload_queues_are_process_isolated(self):
    params = Params()
    params.put("AthenadUploadQueue", [], block=True)
    params.put("SunnylinkUploadQueue", [], block=True)
    ctx = multiprocessing.get_context("spawn")

    for param_key, source_backend in (("AthenadUploadQueue", "comma"), ("SunnylinkUploadQueue", "sunnylink")):
      process = ctx.Process(target=_cache_upload, args=(param_key, source_backend))
      process.start()
      process.join(10)
      if process.is_alive():
        process.kill()
        process.join()
      assert process.exitcode == 0

    assert [item["id"] for item in params.get("AthenadUploadQueue")] == ["comma"]
    assert [item["id"] for item in params.get("SunnylinkUploadQueue")] == ["sunnylink"]

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
