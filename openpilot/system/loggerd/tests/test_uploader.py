import io
import os
import tempfile
import threading
import logging
import json
from pathlib import Path
from types import SimpleNamespace
from openpilot.common.hardware.hw import Paths

from openpilot.common.swaglog import cloudlog
import openpilot.system.loggerd.uploader as uploader
from openpilot.common.api.backend import COMMA_BACKEND, KONIK_BACKEND
from openpilot.system.loggerd.uploader import clear_locks, main, Uploader, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE

from openpilot.system.loggerd.tests.loggerd_tests_common import UploaderTestCase


# OpenpilotTestCase resolves fixtures from module-level functions
def tmp_path():
  with tempfile.TemporaryDirectory() as d:
    yield Path(d)


class FakeLogHandler(logging.Handler):
  def __init__(self):
    logging.Handler.__init__(self)
    self.condition = threading.Condition()
    self.reset()

  def reset(self):
    with self.condition:
      self.upload_order = []
      self.upload_ignored = []

  def emit(self, record):
    try:
      j = json.loads(record.getMessage())
      with self.condition:
        if j["event"] == "upload_success":
          self.upload_order.append(j["key"])
        if j["event"] == "upload_ignored":
          self.upload_ignored.append(j["key"])
        self.condition.notify_all()
    except Exception:
      pass

  def wait_for_uploads(self, count: int, ignored: bool = False):
    uploads = self.upload_ignored if ignored else self.upload_order
    with self.condition:
      assert self.condition.wait_for(lambda: len(uploads) >= count, timeout=1), "Uploader did not process all files"

log_handler = FakeLogHandler()
cloudlog.addHandler(log_handler)


class TestUploader(UploaderTestCase):
  def setup_method(self):
    super().openpilot_setup_method()
    log_handler.reset()

  def start_thread(self):
    self.end_event = threading.Event()
    self.up_thread = threading.Thread(target=main, args=[self.end_event])
    self.up_thread.daemon = True
    self.up_thread.start()

  def join_thread(self):
    self.end_event.set()
    self.up_thread.join()

  def gen_files(self, lock=False, xattr: bytes | None = None, boot=True) -> list[Path]:
    f_paths = []
    for t in ["qlog", "rlog", "dcamera.hevc", "fcamera.hevc"]:
      f_paths.append(self.make_file_with_data(self.seg_dir, t, 1, lock=lock, upload_xattr=xattr))

    if boot:
      f_paths.append(self.make_file_with_data("boot", f"{self.seg_dir}", 1, lock=lock, upload_xattr=xattr))
    return f_paths

  def gen_order(self, seg1: list[int], seg2: list[int], boot=True) -> list[str]:
    keys = []
    if boot:
      keys += [f"boot/{self.seg_format.format(i)}.zst" for i in seg1]
      keys += [f"boot/{self.seg_format2.format(i)}.zst" for i in seg2]
    keys += [f"{self.seg_format.format(i)}/qlog.zst" for i in seg1]
    keys += [f"{self.seg_format2.format(i)}/qlog.zst" for i in seg2]
    return keys

  def test_upload(self):
    self.gen_files(lock=False)
    exp_order = self.gen_order([self.seg_num], [])

    self.start_thread()
    log_handler.wait_for_uploads(len(exp_order))
    self.join_thread()

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_upload_with_wrong_xattr(self):
    self.gen_files(lock=False, xattr=b'0')
    exp_order = self.gen_order([self.seg_num], [])

    self.start_thread()
    log_handler.wait_for_uploads(len(exp_order))
    self.join_thread()

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_upload_ignored(self):
    self.set_ignore()
    self.gen_files(lock=False)
    exp_order = self.gen_order([self.seg_num], [])

    self.start_thread()
    log_handler.wait_for_uploads(len(exp_order), ignored=True)
    self.join_thread()

    assert len(log_handler.upload_order) == 0, "Some files were not ignored"
    assert not len(log_handler.upload_ignored) < len(exp_order), "Some files failed to ignore"
    assert not len(log_handler.upload_ignored) > len(exp_order), "Some files were ignored twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not ignored"

    assert log_handler.upload_ignored == exp_order, "Files ignored in wrong order"

  def test_upload_files_in_create_order(self):
    seg1_nums = [0, 1, 2, 10, 20]
    for i in seg1_nums:
      self.seg_dir = self.seg_format.format(i)
      self.gen_files(boot=False)
    seg2_nums = [5, 50, 51]
    for i in seg2_nums:
      self.seg_dir = self.seg_format2.format(i)
      self.gen_files(boot=False)

    exp_order = self.gen_order(seg1_nums, seg2_nums, boot=False)

    self.start_thread()
    log_handler.wait_for_uploads(len(exp_order))
    self.join_thread()

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_no_upload_with_lock_file(self):
    f_paths = self.gen_files(lock=True, boot=False)
    uploader = Uploader("0000000000000000", Paths.log_root())

    for f_path in f_paths:
      fn = f_path.with_suffix(f_path.suffix.replace(".zst", ""))
      assert all(candidate[2] != str(fn) for candidate in uploader.list_upload_files(metered=False)), "Locked file selected for upload"

  def test_no_upload_with_xattr(self):
    f_paths = self.gen_files(lock=False, xattr=UPLOAD_ATTR_VALUE)
    uploader = Uploader("0000000000000000", Paths.log_root())
    upload_candidates = {candidate[2] for candidate in uploader.list_upload_files(metered=False)}
    assert upload_candidates.isdisjoint(map(str, f_paths)), "Uploaded file selected again"

  def test_clear_locks_on_startup(self, mocker):
    f_paths = self.gen_files(lock=True, boot=False)
    locks_cleared = threading.Event()

    def clear_locks_and_signal(root):
      clear_locks(root)
      locks_cleared.set()

    mocker.patch("openpilot.system.loggerd.uploader.clear_locks", side_effect=clear_locks_and_signal)
    self.start_thread()
    assert locks_cleared.wait(timeout=1), "Uploader did not clear locks on startup"
    self.join_thread()

    for f_path in f_paths:
      lock_path = f_path.with_suffix(f_path.suffix + ".lock")
      assert not lock_path.is_file(), "File lock not cleared on startup"

  def test_lock_after_construction_uses_konik_only(self, monkeypatch):
    calls = []

    class Api:
      def __init__(self, config, dongle_id):
        self.config = config
        self.dongle_id = dongle_id

      def get_token(self):
        calls.append((self.config.name, "token"))
        return "token"

      def get(self, *args, **kwargs):
        calls.append((self.config.name, "get"))
        return SimpleNamespace(status_code=200, text='{"url": "https://storage.example/upload", "headers": {}}')

    clients = {
      COMMA_BACKEND: Api(COMMA_BACKEND, "comma-id"),
      KONIK_BACKEND: Api(KONIK_BACKEND, "konik-id"),
    }
    monkeypatch.setattr(uploader, "connect_client", lambda params: (KONIK_BACKEND, clients[KONIK_BACKEND]))
    monkeypatch.setattr(uploader, "backend_config", lambda params: KONIK_BACKEND)
    monkeypatch.setattr(uploader, "fake_upload", True)

    Uploader("comma-id", str(Paths.log_root())).do_upload("route/qlog", __file__)
    assert calls == [("konik", "token"), ("konik", "get")]

  def test_backend_change_while_minting_token_aborts_before_signed_url(self, monkeypatch):
    state = {"config": COMMA_BACKEND}

    class Api:
      dongle_id = "comma-id"

      def get_token(self):
        state["config"] = KONIK_BACKEND
        return "comma-token"

      def get(self, *args, **kwargs):
        raise AssertionError("signed URL request must not run")

    monkeypatch.setattr(uploader, "connect_client", lambda params: (COMMA_BACKEND, Api()))
    monkeypatch.setattr(uploader, "backend_config", lambda params: state["config"])

    assert Uploader("comma-id", str(Paths.log_root())).do_upload("route/qlog", __file__) is None

  def test_backend_change_after_signed_url_aborts_before_put(self, monkeypatch, tmp_path):
    state = {"config": COMMA_BACKEND}
    monkeypatch.setattr(uploader.requests, "put", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("PUT must not run")))

    class Api:
      dongle_id = "comma-id"

      def get_token(self):
        return "token"

      def get(self, *args, **kwargs):
        state["config"] = KONIK_BACKEND
        return SimpleNamespace(status_code=200, text='{"url": "https://storage.example/upload", "headers": {}}')

    monkeypatch.setattr(uploader, "connect_client", lambda params: (COMMA_BACKEND, Api()))
    monkeypatch.setattr(uploader, "backend_config", lambda params: state["config"])
    monkeypatch.setattr(uploader, "fake_upload", False)
    fn = tmp_path / "qlog"
    fn.write_bytes(b"route")

    assert Uploader("comma-id", str(tmp_path)).do_upload("route/qlog", str(fn)) is None

  def test_backend_change_with_ignored_response_keeps_file_for_retry(self, monkeypatch, tmp_path):
    state = {"config": COMMA_BACKEND}

    class Api:
      dongle_id = "comma-id"

      def get_token(self):
        return "token"

      def get(self, *args, **kwargs):
        state["config"] = KONIK_BACKEND
        return SimpleNamespace(status_code=412)

    monkeypatch.setattr(uploader, "connect_client", lambda params: (COMMA_BACKEND, Api()))
    monkeypatch.setattr(uploader, "backend_config", lambda params: state["config"])
    fn = tmp_path / "qlog"
    fn.write_bytes(b"route")

    assert Uploader("comma-id", str(tmp_path)).upload("qlog", "route/qlog", str(fn), 1, False) is False
    assert UPLOAD_ATTR_NAME not in os.listxattr(fn)

  def test_failed_transition_retries_file_through_konik_cdn(self, monkeypatch, tmp_path):
    state = {"config": COMMA_BACKEND, "switch_during_get": True}
    requests_seen = []
    puts = []

    class Api:
      def __init__(self, config):
        self.config = config
        self.dongle_id = f"{config.name}-id"

      def get_token(self):
        return f"{self.config.name}-token"

      def get(self, endpoint, **kwargs):
        requests_seen.append((self.config.name, endpoint, kwargs["access_token"]))
        if state["switch_during_get"]:
          state["config"] = KONIK_BACKEND
          state["switch_during_get"] = False
        return SimpleNamespace(status_code=200, text=json.dumps({
          "url": f"https://{self.config.name}-cdn.example/upload", "headers": {},
        }))

    monkeypatch.setattr(uploader, "connect_client", lambda params: (state["config"], Api(state["config"])))
    monkeypatch.setattr(uploader, "backend_config", lambda params: state["config"])
    monkeypatch.setattr(uploader.requests, "put", lambda url, **kwargs: puts.append(url) or SimpleNamespace(
      status_code=200, request=SimpleNamespace(headers={"Content-Length": "5"})))
    monkeypatch.setattr(uploader, "fake_upload", False)
    fn = tmp_path / "qlog"
    fn.write_bytes(b"route")
    instance = Uploader("comma-id", str(tmp_path))

    assert instance.upload("qlog", "route/qlog", str(fn), 1, False) is False
    assert UPLOAD_ATTR_NAME not in os.listxattr(fn)
    assert instance.upload("qlog", "route/qlog", str(fn), 1, False) is True
    assert os.getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
    assert requests_seen == [
      ("comma", "v1.4/comma-id/upload_url/", "comma-token"),
      ("konik", "v1.4/konik-id/upload_url/", "konik-token"),
    ]
    assert puts == ["https://konik-cdn.example/upload"]

  def test_backend_change_while_opening_stream_keeps_file_for_retry(self, monkeypatch, tmp_path):
    state = {"config": COMMA_BACKEND}

    class Api:
      dongle_id = "comma-id"

      def get_token(self):
        return "comma-token"

      def get(self, *args, **kwargs):
        return SimpleNamespace(status_code=200, text='{"url": "https://storage.example/upload", "headers": {}}')

    def get_upload_stream(*args, **kwargs):
      state["config"] = KONIK_BACKEND
      return io.BytesIO(b"route"), 5

    monkeypatch.setattr(uploader, "connect_client", lambda params: (COMMA_BACKEND, Api()))
    monkeypatch.setattr(uploader, "backend_config", lambda params: state["config"])
    monkeypatch.setattr(uploader, "get_upload_stream", get_upload_stream)
    monkeypatch.setattr(uploader.requests, "put", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("PUT must not run")))
    monkeypatch.setattr(uploader, "fake_upload", False)
    fn = tmp_path / "qlog"
    fn.write_bytes(b"route")

    assert Uploader("comma-id", str(tmp_path)).upload("qlog", "route/qlog", str(fn), 1, False) is False
    assert UPLOAD_ATTR_NAME not in os.listxattr(fn)

  def test_backend_change_during_put_keeps_file_for_retry(self, monkeypatch, tmp_path):
    state = {"config": COMMA_BACKEND}

    class Api:
      dongle_id = "comma-id"

      def get_token(self):
        return "comma-token"

      def get(self, *args, **kwargs):
        return SimpleNamespace(status_code=200, text='{"url": "https://storage.example/upload", "headers": {}}')

    def put(*args, **kwargs):
      state["config"] = KONIK_BACKEND
      return SimpleNamespace(status_code=200)

    monkeypatch.setattr(uploader, "connect_client", lambda params: (COMMA_BACKEND, Api()))
    monkeypatch.setattr(uploader, "backend_config", lambda params: state["config"])
    monkeypatch.setattr(uploader.requests, "put", put)
    monkeypatch.setattr(uploader, "fake_upload", False)
    fn = tmp_path / "qlog"
    fn.write_bytes(b"route")

    assert Uploader("comma-id", str(tmp_path)).upload("qlog", "route/qlog", str(fn), 1, False) is False
    assert UPLOAD_ATTR_NAME not in os.listxattr(fn)

  def test_backend_change_after_success_status_keeps_file_for_retry(self, monkeypatch, tmp_path):
    checks = 0

    class Api:
      dongle_id = "comma-id"

      def get_token(self):
        return "comma-token"

      def get(self, *args, **kwargs):
        return SimpleNamespace(status_code=200, text='{"url": "https://storage.example/upload", "headers": {}}')

    def config(_):
      nonlocal checks
      checks += 1
      return COMMA_BACKEND if checks <= 5 else KONIK_BACKEND

    monkeypatch.setattr(uploader, "connect_client", lambda params: (COMMA_BACKEND, Api()))
    monkeypatch.setattr(uploader, "backend_config", config)
    monkeypatch.setattr(uploader.requests, "put", lambda *args, **kwargs: SimpleNamespace(
      status_code=200, request=SimpleNamespace(headers={"Content-Length": "5"})))
    monkeypatch.setattr(uploader, "fake_upload", False)
    fn = tmp_path / "qlog"
    fn.write_bytes(b"route")

    assert Uploader("comma-id", str(tmp_path)).upload("qlog", "route/qlog", str(fn), 1, False) is False
    assert UPLOAD_ATTR_NAME not in os.listxattr(fn)
