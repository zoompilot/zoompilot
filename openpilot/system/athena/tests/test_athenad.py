from functools import wraps
import json
import multiprocessing
import os
import pytest
import requests
import shutil
import socket
import time
import threading
import queue
import tempfile
from dataclasses import asdict, replace
from datetime import datetime, timedelta
from pathlib import Path

from websocket import ABNF
from websocket._exceptions import WebSocketConnectionClosedException

from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import messaging

from openpilot.common.api.backend import COMMA_BACKEND, KONIK_BACKEND
from openpilot.common.params import Params
from openpilot.common.timeout import Timeout
from openpilot.system.athena import athenad
from openpilot.system.athena.athenad import MAX_RETRY_COUNT, UPLOAD_SESS, dispatcher
from openpilot.system.athena.rpc import INVALID_PARAMS
from openpilot.system.athena.tests.helpers import HTTPRequestHandler, MockWebsocket, MockApi, EchoSocket
from openpilot.selfdrive.test.helpers import http_server_context
from openpilot.common.hardware.hw import Paths


def seed_athena_server(host, port):
  with Timeout(2, 'HTTP Server seeding failed'):
    while True:
      try:
        UPLOAD_SESS.put(f'http://{host}:{port}/qlog.zst', data='', timeout=10)
        break
      except requests.exceptions.ConnectionError:
        time.sleep(0.1)

def with_upload_handler(func):
  @wraps(func)
  def wrapper(*args, **kwargs):
    end_event = threading.Event()
    thread = threading.Thread(target=athenad.upload_handler, args=(end_event, "comma"))
    thread.start()
    try:
      return func(*args, **kwargs)
    finally:
      end_event.set()
      thread.join()
  return wrapper

def mock_create_connection(mocker):
  return mocker.patch('openpilot.system.athena.athenad.create_connection')


# OpenpilotTestCase resolves fixtures from module-level functions
def tmp_path():
  with tempfile.TemporaryDirectory() as d:
    yield Path(d)

def host():
  with http_server_context(handler=HTTPRequestHandler, setup=seed_athena_server) as (host, port):
    yield f"http://{host}:{port}"

class TestAthenadMethods(OpenpilotTestCase):
  @classmethod
  def setup_class(cls):
    cls.SOCKET_PORT = 45454
    athenad.Api = MockApi  # ty: ignore[invalid-assignment]  # test double
    athenad.LOCAL_PORT_WHITELIST = {cls.SOCKET_PORT}

  def setup_method(self):
    self.default_params = {
      "DongleId": "0000000000000000",
      "CommaDongleId": "0000000000000000",
      "KonikDongleId": "1111111111111111",
      "GithubSshKeys": "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQC307aE+nuHzTAgaJhzSf5v7ZZQW9gaperjhCmyPyl4PzY7T1mDGenTlVTN7yoVFZ9UfO9oMQqo0n1OwDIiqbIFxqnhrHU0cYfj88rI85m5BEKlNu5RdaVTj1tcbaPpQc5kZEolaI1nDDjzV0lwS7jo5VYDHseiJHlik3HH1SgtdtsuamGR2T80q1SyW+5rHoMOJG73IH2553NnWuikKiuikGHUYBd00K1ilVAK2xSiMWJp55tQfZ0ecr9QjEsJ+J/efL4HqGNXhffxvypCXvbUYAFSddOwXUPo5BTKevpxMtH+2YrkpSjocWA04VnTYFiPG6U4ItKmbLOTFZtPzoez private", # noqa: E501
      "GithubUsername": "commaci",
      "AthenadUploadQueue": [],
    }

    self.params = Params()
    for k, v in self.default_params.items():
      self.params.put(k, v, block=True)
    self.params.put_bool("GsmMetered", True, block=True)

    athenad.upload_queue = queue.PriorityQueue()
    athenad.UploadQueueCache.configure("AthenadUploadQueue")
    athenad.cur_upload_items.clear()
    athenad.cancelled_uploads.clear()

    for i in os.listdir(Paths.log_root()):
      p = os.path.join(Paths.log_root(), i)
      if os.path.isdir(p):
        shutil.rmtree(p)
      else:
        os.unlink(p)

  # *** test helpers ***

  @staticmethod
  def _wait_for_upload():
    now = time.monotonic()
    while time.monotonic() - now < 5:
      if athenad.upload_queue.qsize() == 0:
        break

  @staticmethod
  def _create_file(file: str, parent: str | None = None, data: bytes = b'') -> str:
    fn = os.path.join(Paths.log_root() if parent is None else parent, file)
    os.makedirs(os.path.dirname(fn), exist_ok=True)
    with open(fn, 'wb') as f:
      f.write(data)
    return fn

  @staticmethod
  def _video_clips(clip):
    clips = object.__new__(athenad.VideoClips)
    clips.lock = threading.Condition()
    clips.clips = {clip.filename: clip}
    clips.transcode_proc = None
    return clips


  # *** test cases ***

  def test_echo(self):
    assert dispatcher["echo"]("bob") == "bob"

  def test_main_reconnect_uses_coherent_backend_snapshot(self, mocker):
    exit_event = threading.Event()
    clients = []
    for dongle_id, token in (("comma-id", "comma-jwt"), ("konik-id", "konik-jwt")):
      client = mocker.Mock(dongle_id=dongle_id)
      client.get_token.return_value = token
      clients.append(client)

    connect_client = mocker.patch.object(athenad, "connect_client", side_effect=[
      (COMMA_BACKEND, clients[0]),
      (KONIK_BACKEND, clients[1]),
    ])
    websockets = [mocker.Mock(), mocker.Mock()]
    create_connection = mocker.patch.object(athenad, "create_connection", side_effect=websockets)
    snapshots = []

    def handle_long_poll(_ws, event, source_backend, api):
      snapshots.append((source_backend, api))
      if source_backend == "comma":
        self.params.put_bool("UseKonikServer", True, block=True)
      else:
        event.set()

    mocker.patch.object(athenad, "handle_long_poll", side_effect=handle_long_poll)
    mocker.patch.object(athenad, "set_core_affinity")
    mocker.patch.object(athenad, "backoff", return_value=0)

    athenad.main(exit_event)

    assert connect_client.call_count == 2
    assert snapshots == [("comma", clients[0]), ("konik", clients[1])]
    assert [(call.args[0], call.kwargs["cookie"]) for call in create_connection.call_args_list] == [
      ("wss://athena.comma.ai/ws/v2/comma-id", "jwt=comma-jwt"),
      ("wss://athena.konik.ai/ws/v2/konik-id", "jwt=konik-jwt"),
    ]
    assert all(client.get_token.call_count == 1 for client in clients)

  def test_ws_manage_ends_session_on_backend_switch(self, mocker):
    end_event = threading.Event()
    ws = mocker.Mock(sock=None)
    thread = threading.Thread(target=athenad.ws_manage, args=(ws, end_event, "comma"))
    thread.start()

    start = time.monotonic()
    self.params.put_bool("UseKonikServer", True, block=True)
    assert end_event.wait(1.1)
    assert time.monotonic() - start <= 1.1
    thread.join(timeout=1)
    assert not thread.is_alive()
    ws.close.assert_called_once_with()

  def test_reconnect_does_not_process_delayed_old_backend_frame(self, mocker):
    # OpenpilotTestCase resolves params itself; loop instead of parametrize
    for termination in ("backend", "event"):
      self.params.put_bool("UseKonikServer", False, block=True)
      waiting = threading.Event()

      class BlockingQueue(queue.Queue[str]):
        def get(self, *args, **kwargs):
          waiting.set()
          return super().get(*args, **kwargs)

      old_recv_queue = BlockingQueue()
      old_send_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
      handle = mocker.patch.object(athenad, "handle")
      end_event = threading.Event()
      thread = threading.Thread(target=athenad.jsonrpc_handler,
                                args=(end_event, None, "comma", None, old_recv_queue, old_send_queue))
      thread.start()
      assert waiting.wait(1)

      if termination == "backend":
        self.params.put_bool("UseKonikServer", True, block=True)
      else:
        end_event.set()
      old_recv_queue.put_nowait(json.dumps({"method": "uploadFileToUrl", "params": {
        "fn": "old-qlog.zst", "url": "https://comma.example/old-qlog.zst", "headers": {},
      }, "jsonrpc": "2.0", "id": 1}))
      thread.join(timeout=2)

      assert not thread.is_alive()
      handle.assert_not_called()
      assert old_send_queue.empty()
      assert athenad.upload_queue.empty()

  def test_jsonrpc_cannot_override_session_backend(self, mocker):
    source_queue: queue.Queue[str] = queue.Queue()
    target_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
    start_local_proxy = mocker.patch.object(athenad, "startLocalProxy")
    source_queue.put_nowait(json.dumps({"method": "startLocalProxy", "params": {
      "remote_ws_uri": "wss://proxy.example", "local_port": self.SOCKET_PORT, "source_backend": "konik",
    }, "jsonrpc": "2.0", "id": 1}))

    end_event = threading.Event()
    thread = threading.Thread(target=athenad.jsonrpc_handler,
                              args=(end_event, None, "comma", mocker.Mock(), source_queue, target_queue))
    thread.start()
    try:
      _, _, response, _ = target_queue.get(timeout=3)
      assert json.loads(response)["error"]["code"] == INVALID_PARAMS
      start_local_proxy.assert_not_called()
    finally:
      end_event.set()
      thread.join(timeout=2)

  def test_proxy_threads_observe_session_end(self, mocker):
    global_end_event = threading.Event()

    recv_sock, recv_peer = socket.socketpair()
    recv_ws = mocker.Mock(sock=recv_sock)
    recv_end_event = threading.Event()
    recv_thread = threading.Thread(target=athenad.ws_proxy_recv,
                                   args=(recv_ws, mocker.Mock(), mocker.Mock(), recv_end_event, global_end_event))
    recv_thread.start()

    local_sock, local_peer = socket.socketpair()
    signal_sock, signal_peer = socket.socketpair()
    send_end_event = threading.Event()
    send_thread = threading.Thread(target=athenad.ws_proxy_send,
                                   args=(mocker.Mock(), local_sock, signal_sock, send_end_event, global_end_event))
    send_thread.start()
    try:
      global_end_event.set()
      recv_thread.join(timeout=0.5)
      send_thread.join(timeout=0.5)
      assert not recv_thread.is_alive()
      assert not send_thread.is_alive()
    finally:
      recv_sock.close()
      recv_peer.close()
      local_sock.close()
      local_peer.close()
      signal_sock.close()
      signal_peer.close()

  def test_proxy_session_end_interrupts_partial_websocket_frame(self, mocker):
    global_end_event = threading.Event()
    readable_sock, readable_peer = socket.socketpair()
    local_sock, local_peer = socket.socketpair()
    signal_sock, signal_peer = socket.socketpair()
    recv_started = threading.Event()
    closed = threading.Event()

    class PartialFrameWebsocket:
      sock = readable_sock

      def recv(self):
        recv_started.set()
        assert closed.wait(1)
        return b"stale"

      def close(self):
        closed.set()

    ws = PartialFrameWebsocket()
    recv_thread = threading.Thread(target=athenad.ws_proxy_recv,
                                   args=(ws, local_sock, signal_peer, threading.Event(), global_end_event))
    send_thread = threading.Thread(target=athenad.ws_proxy_send,
                                   args=(ws, local_peer, signal_sock, threading.Event(), global_end_event))
    recv_thread.start()
    send_thread.start()
    try:
      readable_peer.send(b"partial")
      assert recv_started.wait(1)
      global_end_event.set()
      recv_thread.join(timeout=1)
      send_thread.join(timeout=1)
      assert not recv_thread.is_alive()
      assert not send_thread.is_alive()
      assert closed.is_set()
    finally:
      for sock in (readable_sock, readable_peer, local_sock, local_peer, signal_sock, signal_peer):
        try:
          sock.close()
        except OSError:
          pass

  def test_get_message(self):
    with self.assertRaises(TimeoutError) as _:
      dispatcher["getMessage"]("controlsState")

    end_event = multiprocessing.Event()

    pub_sock = messaging.pub_sock("deviceState")

    def send_deviceState():
      while not end_event.is_set():
        msg = messaging.new_message('deviceState')
        pub_sock.send(msg.to_bytes())
        time.sleep(0.01)

    p = multiprocessing.Process(target=send_deviceState)
    p.start()
    time.sleep(0.1)
    try:
      deviceState = dispatcher["getMessage"]("deviceState")
      assert deviceState['deviceState']
    finally:
      end_event.set()
      p.join()

  def test_list_data_directory(self):
    route = '2021-03-29--13-32-47'
    segments = [0, 1, 2, 3, 11]

    filenames = ['qlog.zst', 'qcamera.ts', 'rlog.zst', 'fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc']
    files = [f'{route}--{s}/{f}' for s in segments for f in filenames]
    for file in files:
      self._create_file(file)

    resp = dispatcher["listDataDirectory"]()
    assert resp, 'list empty!'
    assert len(resp) == len(files)

    resp = dispatcher["listDataDirectory"](f'{route}--123')
    assert len(resp) == 0

    prefix = f'{route}'
    expected = list(filter(lambda f: f.startswith(prefix), files))
    resp = dispatcher["listDataDirectory"](prefix)
    assert resp, 'list empty!'
    assert len(resp) == len(expected)

    prefix = f'{route}--1'
    expected = list(filter(lambda f: f.startswith(prefix), files))
    resp = dispatcher["listDataDirectory"](prefix)
    assert resp, 'list empty!'
    assert len(resp) == len(expected)

    prefix = f'{route}--1/'
    expected = list(filter(lambda f: f.startswith(prefix), files))
    resp = dispatcher["listDataDirectory"](prefix)
    assert resp, 'list empty!'
    assert len(resp) == len(expected)

    prefix = f'{route}--1/q'
    expected = list(filter(lambda f: f.startswith(prefix), files))
    resp = dispatcher["listDataDirectory"](prefix)
    assert resp, 'list empty!'
    assert len(resp) == len(expected)

  def test_video_clip_hardware_encoder(self, mocker):
    clip = athenad.VideoClips.Clip("route", "fcamera.hevc", 10, 130, 2, 4, "clip.mp4", 123)
    clips = self._video_clips(clip)
    process = mocker.Mock(stdin=None, returncode=0)
    process.poll.return_value = 0
    popen = mocker.patch("openpilot.system.athena.athenad.subprocess.Popen", return_value=process)
    mocker.patch.object(athenad, "PC", False)

    clips._encode(clip, ["segment0", "segment1"], "output.mp4", 10, 120)

    metadata = json.dumps(asdict(clip), separators=(',', ':'))
    assert popen.call_args.args[0] == [
      os.path.join(athenad.BASEDIR, "openpilot/system/loggerd/encoderd"), "--clip", "output.mp4", "10", "120",
      "--bitrate", "2000000", "--speedup", "4", "--metadata", metadata, "--", "segment0", "segment1",
    ]
    assert popen.call_args.kwargs["stdin"] == athenad.subprocess.DEVNULL
    assert clips.transcode_proc is None

  def test_video_clip_hardware_encoder_failure(self, mocker):
    clip = athenad.VideoClips.Clip("route", "fcamera.hevc", 0, 60, 1, 1, "clip.mp4", 123)
    clips = self._video_clips(clip)
    process = mocker.Mock(stdin=None, returncode=1)
    process.poll.return_value = 1
    mocker.patch("openpilot.system.athena.athenad.subprocess.Popen", return_value=process)
    mocker.patch.object(athenad, "PC", False)

    with self.assertRaisesRegex(RuntimeError, "clip encoder exited with code 1"):
      clips._encode(clip, ["segment"], "output.mp4", 0, 60)
    assert clips.transcode_proc is None

  def test_video_clip_software_fallback(self, mocker):
    clip = athenad.VideoClips.Clip("route", "fcamera.hevc", 10, 30, 3, 2, "clip.mp4", 123)
    clips = self._video_clips(clip)
    stdin = mocker.Mock()
    process = mocker.Mock(stdin=stdin, returncode=0)
    process.poll.return_value = 0
    popen = mocker.patch("openpilot.system.athena.athenad.subprocess.Popen", return_value=process)
    mocker.patch.object(athenad, "PC", True)

    clips._encode(clip, ["segment'0", "segment1"], "output.mp4", 10, 20)

    command = popen.call_args.args[0]
    assert ["-r", "40"] == command[command.index("-r"):command.index("-r") + 2]
    assert ["-ss", "5.0"] == command[command.index("-ss"):command.index("-ss") + 2]
    assert ["-t", "10.0"] == command[command.index("-t"):command.index("-t") + 2]
    assert ["-b:v", "3M"] == command[command.index("-b:v"):command.index("-b:v") + 2]
    writes = [call.args[0] for call in stdin.write.call_args_list]
    assert "file 'file:segment'\\''0'\n" in writes[1]
    assert writes[-1].startswith("file 'file:segment1'")

  def test_strip_extension(self):
    # any requested log file with an invalid extension won't return as existing
    fn = self._create_file('qlog.bz2')
    if fn.endswith('.bz2'):
      assert athenad.strip_zst_extension(fn) == fn

    fn = self._create_file('qlog.zst')
    if fn.endswith('.zst'):
      assert athenad.strip_zst_extension(fn) == fn[:-4]

  @parameterized.expand([True, False], names=("compress",))
  def test_do_upload(self, host, compress):
    # random bytes to ensure rather large object post-compression
    fn = self._create_file('qlog', data=os.urandom(10000 * 1024))

    upload_fn = fn + ('.zst' if compress else '')
    item = athenad.UploadItem(path=upload_fn, url="http://localhost:1238", headers={}, created_at=int(time.time()*1000), id='')  # noqa: TID251
    with self.assertRaises(requests.exceptions.ConnectionError):
      athenad._do_upload(item)

    item = athenad.UploadItem(path=upload_fn, url=f"{host}/qlog.zst", headers={}, created_at=int(time.time()*1000), id='')  # noqa: TID251
    resp = athenad._do_upload(item)
    assert resp.status_code == 201

  def test_upload_file_to_url(self, host):
    fn = self._create_file('qlog.zst')

    resp = dispatcher["uploadFileToUrl"]("qlog.zst", f"{host}/qlog.zst", {})
    assert resp['enqueued'] == 1
    assert 'failed' not in resp
    assert {"path": fn, "url": f"{host}/qlog.zst", "headers": {}}.items() <= resp['items'][0].items()
    assert resp['items'][0].get('id') is not None
    assert athenad.upload_queue.qsize() == 1

  def test_upload_file_to_url_duplicate(self, host):
    self._create_file('qlog.zst')

    url1 = f"{host}/qlog.zst?sig=sig1"
    dispatcher["uploadFileToUrl"]("qlog.zst", url1, {})

    # Upload same file again, but with different signature
    url2 = f"{host}/qlog.zst?sig=sig2"
    resp = dispatcher["uploadFileToUrl"]("qlog.zst", url2, {})
    assert resp == {'enqueued': 0, 'items': []}

  def test_upload_file_to_url_does_not_exist(self, host):
    not_exists_resp = dispatcher["uploadFileToUrl"]("does_not_exist.zst", "http://localhost:1238", {})
    assert not_exists_resp == {'enqueued': 0, 'items': [], 'failed': ['does_not_exist.zst']}

  @with_upload_handler
  def test_upload_handler(self, host):
    fn = self._create_file('qlog.zst')
    item = athenad.UploadItem(path=fn, url=f"{host}/qlog.zst", headers={}, created_at=int(time.time()*1000), id='', allow_cellular=True, source_backend="comma")  # noqa: TID251

    athenad.upload_queue.put_nowait(item)
    self._wait_for_upload()
    time.sleep(0.1)

    # TODO: verify that upload actually succeeded
    # TODO: also check that end_event and metered network raises AbortTransferException
    assert athenad.upload_queue.qsize() == 0

  @parameterized.expand([(500,True), (412,False)], names=("status", "retry"))
  @with_upload_handler
  def test_upload_handler_retry(self, mocker, host, status, retry):
    mock_put = mocker.patch('openpilot.system.athena.athenad.UPLOAD_SESS.put')
    mock_put.return_value.__enter__.return_value.status_code = status
    fn = self._create_file('qlog.zst')
    item = athenad.UploadItem(path=fn, url=f"{host}/qlog.zst", headers={}, created_at=int(time.time()*1000), id='', allow_cellular=True, source_backend="comma")  # noqa: TID251

    athenad.upload_queue.put_nowait(item)
    self._wait_for_upload()
    time.sleep(0.1)

    assert athenad.upload_queue.qsize() == (1 if retry else 0)

    if retry:
      assert athenad.upload_queue.get().retry_count == 1

  @with_upload_handler
  def test_upload_handler_timeout(self):
    """When an upload times out or fails to connect it should be placed back in the queue"""
    fn = self._create_file('qlog.zst')
    item = athenad.UploadItem(path=fn, url="http://localhost:44444/qlog.zst", headers={}, created_at=int(time.time()*1000),  # noqa: TID251
                              id='', allow_cellular=True, source_backend="comma")
    item_no_retry = replace(item, retry_count=MAX_RETRY_COUNT)

    athenad.upload_queue.put_nowait(item_no_retry)
    self._wait_for_upload()
    time.sleep(0.1)

    # Check that upload with retry count exceeded is not put back
    assert athenad.upload_queue.qsize() == 0

    athenad.upload_queue.put_nowait(item)
    self._wait_for_upload()
    time.sleep(0.1)

    # Check that upload item was put back in the queue with incremented retry count
    assert athenad.upload_queue.qsize() == 1
    assert athenad.upload_queue.get().retry_count == 1

  def test_backend_switch_drops_enqueue_retry_and_queued_upload(self, mocker):
    fn = self._create_file('qlog.zst')
    item = athenad.UploadItem(path=fn, url="https://upload.example/qlog.zst", headers={}, created_at=int(time.time() * 1000),  # noqa: TID251
                              id="old", allow_cellular=True, source_backend="comma")
    self.params.put_bool("UseKonikServer", True, block=True)

    assert athenad._upload_file_to_url("qlog.zst", item.url, {}, "comma") == {"enqueued": 0, "items": []}

    retry_tid = -1
    athenad.cur_upload_items[retry_tid] = replace(item, current=True)
    athenad.retry_upload(retry_tid, threading.Event(), source_backend="comma")
    assert athenad.cur_upload_items[retry_tid] is None
    assert athenad.upload_queue.empty()

    do_upload = mocker.patch.object(athenad, "_do_upload")
    athenad.upload_queue.put_nowait(item)
    end_event = threading.Event()
    thread = threading.Thread(target=athenad.upload_handler, args=(end_event, "comma"))
    thread.start()
    with Timeout(2, "stale upload was not removed"):
      while not athenad.upload_queue.empty():
        time.sleep(0.01)
    end_event.set()
    thread.join(timeout=2)

    assert not thread.is_alive()
    do_upload.assert_not_called()
    assert self.params.get("AthenadUploadQueue") == []
    assert os.path.exists(fn)

  @with_upload_handler
  def test_cancel_upload(self):
    item = athenad.UploadItem(path="qlog.zst", url="http://localhost:44444/qlog.zst", headers={},
                              created_at=int(time.time()*1000), id='id', allow_cellular=True, source_backend="comma")  # noqa: TID251
    athenad.upload_queue.put_nowait(item)
    dispatcher["cancelUpload"](item.id)

    self._wait_for_upload()
    time.sleep(0.1)

    assert athenad.upload_queue.qsize() == 0
    assert len(athenad.cancelled_uploads) == 0
    assert self.params.get("AthenadUploadQueue") == []

  @with_upload_handler
  def test_cancel_expiry(self):
    t_future = datetime.now() - timedelta(days=40)
    ts = int(t_future.strftime("%s")) * 1000

    # Item that would time out if actually uploaded
    fn = self._create_file('qlog.zst')
    item = athenad.UploadItem(path=fn, url="http://localhost:44444/qlog.zst", headers={}, created_at=ts, id='', allow_cellular=True, source_backend="comma")

    athenad.upload_queue.put_nowait(item)
    self._wait_for_upload()
    time.sleep(0.1)

    assert athenad.upload_queue.qsize() == 0

  def test_list_upload_queue_empty(self):
    items = dispatcher["listUploadQueue"]()
    assert len(items) == 0

  @with_upload_handler
  def test_list_upload_queue_current(self, host: str):
    fn = self._create_file('qlog.zst')
    item = athenad.UploadItem(path=fn, url=f"{host}/qlog.zst", headers={}, created_at=int(time.time()*1000), id='', allow_cellular=True, source_backend="comma")  # noqa: TID251

    athenad.upload_queue.put_nowait(item)
    self._wait_for_upload()

    items = dispatcher["listUploadQueue"]()
    assert len(items) == 1
    assert items[0]['current']

  def test_list_upload_queue_priority(self):
    priorities = (25, 50, 99, 75, 0)

    for i in priorities:
      fn = f'qlog_{i}.zst'
      fp = self._create_file(fn)
      item = athenad.UploadItem(
        path=fp,
        url=f"http://localhost:44444/{fn}",
        headers={},
        created_at=int(time.time()*1000),  # noqa: TID251
        id='',
        allow_cellular=True,
        priority=i,
        source_backend="comma",
      )
      athenad.upload_queue.put_nowait(item)

    for i in sorted(priorities):
      assert athenad.upload_queue.get_nowait().priority == i

  def test_list_upload_queue(self):
    item = athenad.UploadItem(path="qlog.zst", url="http://localhost:44444/qlog.zst", headers={},
                              created_at=int(time.time()*1000), id='id', allow_cellular=True, source_backend="comma")  # noqa: TID251
    athenad.upload_queue.put_nowait(item)

    items = dispatcher["listUploadQueue"]()
    assert len(items) == 1
    assert items[0] == asdict(item)
    assert not items[0]['current']

    assert item.id is not None
    athenad.cancelled_uploads.add(item.id)
    items = dispatcher["listUploadQueue"]()
    assert len(items) == 0

  def test_upload_queue_persistence(self):
    item1 = athenad.UploadItem(path="_", url="_", headers={}, created_at=int(time.time()), id='id1', source_backend="comma")  # noqa: TID251
    item2 = athenad.UploadItem(path="_", url="_", headers={}, created_at=int(time.time()), id='id2', source_backend="comma")  # noqa: TID251

    athenad.upload_queue.put_nowait(item1)
    athenad.upload_queue.put_nowait(item2)

    # Ensure canceled items are not persisted
    assert item2.id is not None
    athenad.cancelled_uploads.add(item2.id)

    # serialize item
    athenad.UploadQueueCache.cache(athenad.upload_queue)

    # deserialize item
    athenad.upload_queue.queue.clear()
    athenad.UploadQueueCache.initialize(athenad.upload_queue, "comma")

    assert athenad.upload_queue.qsize() == 1
    assert asdict(athenad.upload_queue.queue[-1]) == asdict(item1)

  def test_upload_queue_cache_sanitizes_mixed_provenance(self):
    paths = {name: self._create_file(f"{name}.zst") for name in ("current", "mismatch", "legacy", "malformed")}
    now = int(time.time() * 1000)  # noqa: TID251

    def item(name, source_backend):
      return athenad.UploadItem(paths[name], f"https://upload.example/{name}", {}, now, name, source_backend=source_backend)

    current = item("current", "comma")
    mismatched = item("mismatch", "konik")
    legacy = asdict(item("legacy", None))
    legacy.pop("source_backend")
    malformed = {"path": paths["malformed"], "url": "https://upload.example/malformed"}
    self.params.put("AthenadUploadQueue", [asdict(mismatched), legacy, malformed, asdict(current)], block=True)

    athenad.UploadQueueCache.initialize(athenad.upload_queue, "comma")

    assert [asdict(athenad.upload_queue.get_nowait())] == [asdict(current)]
    # the other backend's item is parked, not deleted
    assert self.params.get("AthenadUploadQueue") == [asdict(mismatched), asdict(current)]
    assert all(os.path.exists(path) and athenad.getxattr(path, athenad.LOG_ATTR_NAME) is None for path in paths.values())

  def test_upload_queue_cache_keeps_other_backends_parked_items(self):
    now = int(time.time() * 1000)  # noqa: TID251

    def item(name, source_backend):
      return athenad.UploadItem("_", f"https://upload.example/{name}", {}, now, name, source_backend=source_backend)

    self.params.put("AthenadUploadQueue", [asdict(item("konik_item", "konik")), asdict(item("comma_item", "comma"))], block=True)

    athenad.UploadQueueCache.initialize(athenad.upload_queue, "comma")
    assert athenad.upload_queue.qsize() == 1

    # a routine cache write must not delete the parked item
    athenad.UploadQueueCache.cache(athenad.upload_queue, "comma")
    assert [d["id"] for d in (self.params.get("AthenadUploadQueue") or [])] == ["konik_item", "comma_item"]

    # and switching back makes it runnable again
    athenad.upload_queue.queue.clear()
    athenad.UploadQueueCache.initialize(athenad.upload_queue, "konik")
    assert [athenad.upload_queue.get_nowait().id] == ["konik_item"]

  def test_start_local_proxy(self, mock_create_connection):
    end_event = threading.Event()

    ws_recv = queue.Queue()
    ws_send = queue.Queue()
    mock_ws = MockWebsocket(ws_recv, ws_send)
    mock_create_connection.return_value = mock_ws

    echo_socket = EchoSocket(self.SOCKET_PORT)
    socket_thread = threading.Thread(target=echo_socket.run)
    socket_thread.start()

    athenad.startLocalProxy(end_event, 'ws://localhost:1234', self.SOCKET_PORT)
    assert mock_create_connection.call_args.kwargs["timeout"] == athenad.LOCAL_PROXY_CONNECT_TIMEOUT
    assert mock_ws.timeout == athenad.LOCAL_PROXY_READ_TIMEOUT

    ws_recv.put_nowait(b'ping')
    try:
      recv = ws_send.get(timeout=5)
      assert recv == (b'ping', ABNF.OPCODE_BINARY), recv
    finally:
      # signal websocket close to athenad.ws_proxy_recv
      ws_recv.put_nowait(WebSocketConnectionClosedException())
      socket_thread.join()

  def test_start_local_proxy_rejects_stale_backend_snapshot(self, mocker):
    self.params.put_bool("UseKonikServer", True, block=True)
    api = mocker.Mock()
    create_connection = mocker.patch.object(athenad, "create_connection")

    with pytest.raises(RuntimeError, match="backend changed before"):
      athenad.startLocalProxy(threading.Event(), "ws://localhost:1234", self.SOCKET_PORT, api, "comma")

    api.get_token.assert_not_called()
    create_connection.assert_not_called()

  def test_start_local_proxy_rechecks_backend_after_token(self, mocker):
    self.params.put_bool("UseKonikServer", False, block=True)
    api = mocker.Mock()
    api.get_token.side_effect = lambda: (self.params.put_bool("UseKonikServer", True, block=True) or "comma-jwt")
    create_connection = mocker.patch.object(athenad, "create_connection")

    with pytest.raises(RuntimeError, match="backend changed during"):
      athenad.startLocalProxy(threading.Event(), "ws://localhost:1234", self.SOCKET_PORT, api, "comma")

    api.get_token.assert_called_once_with()
    create_connection.assert_not_called()

  def test_start_local_proxy_bounds_connect_and_aborts_ended_session(self, mocker):
    end_event = threading.Event()
    api = mocker.Mock()
    api.get_token.return_value = "jwt"
    ws = mocker.Mock()

    def end_during_connect(*args, **kwargs):
      end_event.set()
      return ws

    create_connection = mocker.patch.object(athenad, "create_connection", side_effect=end_during_connect)

    with pytest.raises(RuntimeError, match="session or backend changed"):
      athenad.startLocalProxy(end_event, "ws://localhost:1234", self.SOCKET_PORT, api, "comma")

    assert create_connection.call_args.kwargs["timeout"] == athenad.LOCAL_PROXY_CONNECT_TIMEOUT
    ws.settimeout.assert_called_once_with(athenad.LOCAL_PROXY_READ_TIMEOUT)
    ws.close.assert_called_once_with()

  def test_wait_for_send_prefers_session_end(self):
    completion = threading.Event()
    end_event = threading.Event()
    completion.set()
    end_event.set()

    assert not athenad.wait_for_send(completion, end_event)

  def test_ws_send_acknowledges_only_completed_messages(self, mocker):
    send_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
    completion = threading.Event()
    end_event = threading.Event()
    ws = mocker.Mock()
    athenad.send_queue_push("sent", athenad.SEND_PRIORITY_LOW, send_queue, completion)

    thread = threading.Thread(target=athenad.ws_send, args=(ws, end_event, send_queue))
    thread.start()
    try:
      assert completion.wait(1)
      ws.send_frame.assert_called_once()
    finally:
      end_event.set()
      thread.join(timeout=2)

  def test_ws_send_does_not_ack_partial_message(self, mocker):
    send_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
    completion = threading.Event()
    end_event = threading.Event()
    ws = mocker.Mock()
    ws.send_frame.side_effect = [None, OSError("disconnected")]
    athenad.send_queue_push("x" * (athenad.WS_FRAME_SIZE + 1), athenad.SEND_PRIORITY_LOW, send_queue, completion)

    thread = threading.Thread(target=athenad.ws_send, args=(ws, end_event, send_queue))
    thread.start()
    thread.join(timeout=2)

    assert not thread.is_alive()
    assert end_event.is_set()
    assert not completion.is_set()
    assert ws.send_frame.call_count == 2

  def test_ws_send_does_not_ack_if_session_ends_during_final_frame(self, mocker):
    send_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
    completion = threading.Event()
    end_event = threading.Event()
    ws = mocker.Mock()
    ws.send_frame.side_effect = lambda _: end_event.set()
    athenad.send_queue_push("sent", athenad.SEND_PRIORITY_LOW, send_queue, completion)

    thread = threading.Thread(target=athenad.ws_send, args=(ws, end_event, send_queue))
    thread.start()
    thread.join(timeout=2)

    assert not thread.is_alive()
    assert end_event.is_set()
    assert not completion.is_set()

  def test_stat_file_survives_undrained_session_teardown(self, tmp_path):
    stat_path = tmp_path / "stats.json"
    stat_path.write_text("{}")
    send_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
    end_event = threading.Event()
    thread = threading.Thread(target=athenad.stat_handler, args=(end_event, str(tmp_path), False, send_queue))
    thread.start()

    _, _, _, completion = send_queue.get(timeout=1)
    assert completion is not None
    end_event.set()
    thread.join(timeout=2)

    assert not thread.is_alive()
    assert not completion.is_set()
    assert stat_path.exists()

  def test_stat_file_is_deleted_after_transport(self, tmp_path):
    stat_path = tmp_path / "stats.json"
    stat_path.write_text("{}")
    send_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
    end_event = threading.Event()
    thread = threading.Thread(target=athenad.stat_handler, args=(end_event, str(tmp_path), False, send_queue))
    thread.start()
    try:
      _, _, _, completion = send_queue.get(timeout=1)
      assert completion is not None
      completion.set()
      for _ in range(20):
        if not stat_path.exists():
          break
        time.sleep(0.05)
      assert not stat_path.exists()
    finally:
      end_event.set()
      thread.join(timeout=2)

  def test_log_attempt_is_not_marked_before_transport(self, mocker, tmp_path):
    log_path = tmp_path / "swaglog.1"
    log_path.write_text("log")
    mocker.patch.object(athenad, "PC", False)
    mocker.patch.object(athenad.Paths, "swaglog_root", return_value=str(tmp_path))
    mocker.patch.object(athenad, "get_logs_to_send_sorted", return_value=[log_path.name])
    setxattr = mocker.patch.object(athenad, "setxattr")
    send_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
    end_event = threading.Event()
    thread = threading.Thread(target=athenad.log_handler, args=(end_event, athenad.LOG_ATTR_NAME, send_queue))
    thread.start()

    _, _, _, completion = send_queue.get(timeout=1)
    assert completion is not None
    end_event.set()
    thread.join(timeout=2)

    assert not thread.is_alive()
    assert not completion.is_set()
    setxattr.assert_not_called()

  def test_log_attempt_is_marked_after_transport(self, mocker, tmp_path):
    log_path = tmp_path / "swaglog.1"
    log_path.write_text("log")
    mocker.patch.object(athenad, "PC", False)
    mocker.patch.object(athenad.Paths, "swaglog_root", return_value=str(tmp_path))
    mocker.patch.object(athenad, "get_logs_to_send_sorted", return_value=[log_path.name])
    setxattr = mocker.patch.object(athenad, "setxattr")
    send_queue: queue.PriorityQueue[athenad.SendQueueItem] = queue.PriorityQueue()
    end_event = threading.Event()
    thread = threading.Thread(target=athenad.log_handler, args=(end_event, athenad.LOG_ATTR_NAME, send_queue))
    thread.start()
    try:
      _, _, _, completion = send_queue.get(timeout=1)
      assert completion is not None
      completion.set()
      for _ in range(20):
        if setxattr.called:
          break
        time.sleep(0.05)
      assert setxattr.call_count == 1
      assert setxattr.call_args.args[:2] == (str(log_path), athenad.LOG_ATTR_NAME)
    finally:
      end_event.set()
      thread.join(timeout=2)

  def test_get_ssh_authorized_keys(self):
    keys = dispatcher["getSshAuthorizedKeys"]()
    assert keys == self.default_params["GithubSshKeys"]

  def test_get_github_username(self):
    keys = dispatcher["getGithubUsername"]()
    assert keys == self.default_params["GithubUsername"]

  def test_get_version(self):
    resp = dispatcher["getVersion"]()
    keys = ["version", "remote", "branch", "commit", "commit_date"]
    assert list(resp.keys()) == keys
    for k in keys:
      assert isinstance(resp[k], str), f"{k} is not a string"
      assert len(resp[k]) > 0, f"{k} has no value"

  def test_jsonrpc_handler(self):
    end_event = threading.Event()
    thread = threading.Thread(target=athenad.jsonrpc_handler, args=(end_event,))
    thread.daemon = True
    thread.start()
    try:
      # with params
      athenad.recv_queue.put_nowait(json.dumps({"method": "echo", "params": ["hello"], "jsonrpc": "2.0", "id": 0}))
      _, _, resp, _ = athenad.send_queue.get(timeout=3)
      assert json.loads(resp) == {'result': 'hello', 'id': 0, 'jsonrpc': '2.0'}
      # without params
      athenad.recv_queue.put_nowait(json.dumps({"method": "getNetworkType", "jsonrpc": "2.0", "id": 0}))
      _, _, resp, _ = athenad.send_queue.get(timeout=3)
      assert json.loads(resp) == {'result': 1, 'id': 0, 'jsonrpc': '2.0'}
      # log forwarding
      athenad.recv_queue.put_nowait(json.dumps({'result': {'success': 1}, 'id': 0, 'jsonrpc': '2.0'}))
      resp = athenad.log_recv_queue.get(timeout=3)
      assert json.loads(resp) == {'result': {'success': 1}, 'id': 0, 'jsonrpc': '2.0'}
    finally:
      end_event.set()
      thread.join()

  def test_jsonrpc_upload_provenance_cannot_be_spoofed(self):
    self._create_file("qlog.zst")
    end_event = threading.Event()
    thread = threading.Thread(target=athenad.jsonrpc_handler, args=(end_event, None, "comma"))
    thread.start()
    try:
      params = {"fn": "qlog.zst", "url": "https://upload.example/qlog.zst", "headers": {}, "source_backend": "konik"}
      athenad.recv_queue.put_nowait(json.dumps({"method": "uploadFileToUrl", "params": params, "jsonrpc": "2.0", "id": 1}))
      _, _, response, _ = athenad.send_queue.get(timeout=3)
      assert json.loads(response)["error"]["code"] == INVALID_PARAMS
      assert athenad.upload_queue.empty()

      params.pop("source_backend")
      athenad.recv_queue.put_nowait(json.dumps({"method": "uploadFileToUrl", "params": params, "jsonrpc": "2.0", "id": 2}))
      _, _, response, _ = athenad.send_queue.get(timeout=3)
      assert json.loads(response)["result"]["items"][0]["source_backend"] == "comma"
    finally:
      end_event.set()
      thread.join()

  def test_get_logs_to_send_sorted(self):
    fl = []
    for i in range(10):
      file = f'swaglog.{i:010}'
      self._create_file(file, Paths.swaglog_root())
      fl.append(file)

    # ensure the list is all logs except most recent
    sl = athenad.get_logs_to_send_sorted()
    assert sl == fl[:-1]
