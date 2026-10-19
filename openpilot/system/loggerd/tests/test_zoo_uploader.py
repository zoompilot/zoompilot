"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import http.server
import os
import sys
import threading
import types
from datetime import UTC, datetime


class FakeParamsDict:
  # dict-backed params store used two ways: installed as Params by the host
  # stub below, and used directly by these tests on any platform
  def __init__(self):
    self.values = {}

  def get(self, key, **_):
    return self.values.get(key)

  def put(self, key, value, **_):
    self.values[key] = value.encode() if isinstance(value, str) else value

  def get_bool(self, key, **_):
    return bool(self.values.get(key))


# The real stack (cereal, params_pyx) exists on device and CI; on a host Mac
# these stubs activate instead so the zoo uploader logic still gets tested
try:
  import openpilot.cereal.messaging
except Exception:
  cereal = types.ModuleType('openpilot.cereal')
  log_mod = types.ModuleType('openpilot.cereal.log')

  class _NetworkType:
    none = 0
    wifi = 1
    cell2 = 2

  class _NetworkStrength:
    zero = 0
    weak = 1

  class _DeviceState:
    NetworkType = _NetworkType
    NetworkStrength = _NetworkStrength

  log_mod.DeviceState = _DeviceState
  cereal.log = log_mod
  messaging_mod = types.ModuleType('openpilot.cereal.messaging')

  class _FakeSubMaster:
    def __init__(self, *_):
      pass

  messaging_mod.SubMaster = _FakeSubMaster
  cereal.messaging = messaging_mod
  sys.modules['openpilot.cereal'] = cereal
  sys.modules['openpilot.cereal.log'] = log_mod
  sys.modules['openpilot.cereal.messaging'] = messaging_mod

try:
  import openpilot.common.params
except Exception:
  params_mod = types.ModuleType('openpilot.common.params')
  params_mod.Params = FakeParamsDict
  sys.modules['openpilot.common.params'] = params_mod

try:
  import openpilot.common.swaglog  # noqa: F401
except Exception:
  swaglog_mod = types.ModuleType('openpilot.common.swaglog')

  class _Cloudlog:
    def __getattr__(self, name):
      return lambda *a, **k: None

  swaglog_mod.cloudlog = _Cloudlog()
  sys.modules['openpilot.common.swaglog'] = swaglog_mod

from openpilot.common.zoo import pgp
from openpilot.common.zoo import add_upload_request, latest_route_dir, list_route_dirs, load_upload_requests, normalize_route_request
from openpilot.common.zoo import add_current_drive_request
from openpilot.common.zoo.s3sig import S3Config
from openpilot.system.loggerd.uploader import MAX_UPLOAD_SIZES, Uploader, listdir_by_creation
from openpilot.system.loggerd.zoo_uploader import ZooUploader, PRUNE_TRIGGER_BYTES, _clear_stale_locks, zoo_config_from_params
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr


class RecordingStub(http.server.BaseHTTPRequestHandler):
  # returns 200 for everything until told to fail, recording keys and bodies
  responses = {}
  seen = []
  list_xml = b''
  deletions = []

  def do_PUT(self):
    length = int(self.headers.get('Content-Length', 0))
    body = self.rfile.read(length)
    RecordingStub.seen.append((self.path, body))
    status = RecordingStub.responses.get(self.path, 200)
    self.send_response(status)
    if status != 200:
      self.send_header('Content-Type', 'application/xml')
      self.end_headers()
      self.wfile.write(b'<?xml version="1.0"?><Error><Code>SignatureDoesNotMatch</Code></Error>')
    else:
      self.end_headers()

  def do_GET(self):
    self.send_response(200)
    self.send_header('Content-Type', 'application/xml')
    self.send_header('Content-Length', str(len(RecordingStub.list_xml)))
    self.end_headers()
    self.wfile.write(RecordingStub.list_xml)

  def do_DELETE(self):
    RecordingStub.deletions.append(self.path)
    self.send_response(204)
    self.end_headers()

  def log_message(self, *args):
    pass


def make_uploader(tmp_path, params=None, scope='minimal', status_map=None, bucket='bucket', contents='default', encrypt=True):
  params = params or FakeParamsDict()
  if params.get('RouteCount') is None:
    params.put('RouteCount', '0')
  if params.get('BootCount') is None:
    params.put('BootCount', '0')

  # a throwaway cv25519-shaped recipient: a real key is not needed to test
  # uploader logic, so a stub recipient with the fields pgp.encrypt_file uses
  class _FakeKey:
    algo = pgp.ECDH_ALGO

    def key_id(self):
      return b'\x00' * 8

    def fingerprint(self):
      return b'\x11' * 20

  recipient = {'key': _FakeKey(), 'fields': {'point': b'\x40' + b'\x00' * 32, 'hash_id': 8, 'sym_id': 7}}
  # but the real encrypt path needs a real shared secret: monkeypatched below
  # in tests that exercise bytes end to end
  s3 = S3Config(endpoint='http://127.0.0.1:9', region='us-east-1', bucket=bucket, access_key_id='k', secret_access_key='s', path_style=True)
  RecordingStub.responses = status_map or {}
  return ZooUploader(str(tmp_path), s3, 'zooprefix', scope, [recipient], params, contents, encrypt)


def write_segment(root, counter_hex, files, part=1):
  # real loggerd layout: one directory per segment, `<route>--<part>`
  seg = root / f'{counter_hex}--abcdefghij--{part}'
  seg.mkdir(parents=True, exist_ok=True)
  for name, data in files.items():
    (seg / name).write_bytes(data)
  return seg


def list_result_xml(entries):
  contents = ''.join(
    f'<Contents><Key>{k}</Key><Size>{s}</Size><LastModified>{d}</LastModified></Contents>'
    for k, s, d in entries)
  head = '<?xml version="1.0"?><ListBucketResult xmlns="http://s3.amazonaws.com/doc/2006-03-01/"><IsTruncated>false</IsTruncated>'
  return (head + contents + '</ListBucketResult>').encode()


def test_ring_never_prunes_non_route_groups(tmp_path):
  # a stray object under the prefix (the configure.py self-test probe,
  # anything another tool writes) is never a deletion candidate; it
  # occupies the budget like boot/crash do, and the ring takes what it
  # needs from real routes instead
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '400')
  uploader = make_uploader(tmp_path, params=params)

  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'
  RecordingStub.list_xml = list_result_xml([
    ('zooprefix/_selftest/123', 500, '2026-01-01T00:00:00.000Z'),
    ('zooprefix/00000001--abcdefghij/qlog.gpg', 100, '2026-01-02T00:00:00.000Z'),
    ('zooprefix/00000002--abcdefghij/qlog.gpg', 100, '2026-01-03T00:00:00.000Z'),
  ])
  RecordingStub.deletions = []
  try:
    uploader.maybe_prune(force=True)
  finally:
    server.shutdown()

  assert sorted(RecordingStub.deletions) == [
    '/bucket/zooprefix/00000001--abcdefghij/qlog.gpg',
    '/bucket/zooprefix/00000002--abcdefghij/qlog.gpg',
  ]
  assert not any('_selftest' in d for d in RecordingStub.deletions)


def test_stale_locks_clear_only_offroad(tmp_path):
  write_segment(tmp_path, '00000010', {'qlog': b'x'})
  (tmp_path / '00000010--abcdefghij--1' / 'qlog.lock').write_bytes(b'')

  onroad = FakeParamsDict()  # IsOffroad absent = on-road
  _clear_stale_locks(onroad, str(tmp_path))
  assert (tmp_path / '00000010--abcdefghij--1' / 'qlog.lock').exists()

  offroad = FakeParamsDict()
  offroad.put('IsOffroad', '1')
  _clear_stale_locks(offroad, str(tmp_path))
  assert not (tmp_path / '00000010--abcdefghij--1' / 'qlog.lock').exists()


def test_scope_filter(tmp_path):
  uploader = make_uploader(tmp_path)
  write_segment(tmp_path, '00000010', {'qlog': b'q', 'rlog': b'r', 'fcamera.ts': b'f', 'qcamera.ts': b'v'})
  (tmp_path / 'boot').mkdir()
  (tmp_path / 'boot' / '00000002--bootid.zst').write_bytes(b'b')
  (tmp_path / 'crash').mkdir()
  (tmp_path / 'crash' / 'crash1').write_bytes(b'c')

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  names = sorted(k.rsplit('/', 1)[-1] for k in keys)
  assert names == ['00000002--bootid.zst', 'crash1', 'qcamera.ts', 'qlog']
  assert 'rlog' not in names and 'fcamera.ts' not in names

  uploader.scope = 'full'
  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  names = sorted(k.rsplit('/', 1)[-1] for k in keys)
  assert 'rlog' in names and 'fcamera.ts' in names


def test_hidden_dirs_never_upload(tmp_path):
  uploader = make_uploader(tmp_path, scope='full')
  write_segment(tmp_path, '00000010', {'qlog': b'q'})
  (tmp_path / '.zoo-spool' / 'tmpabc.plain').write_bytes(b'in flight')

  # the stock enumeration behind both uploaders and the deleter skips it
  assert listdir_by_creation(str(tmp_path)) == ['00000010--abcdefghij--1']
  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert keys == ['00000010--abcdefghij--1/qlog']
  assert not uploader._in_scope('.zoo-spool/tmpabc.plain')


def test_size_caps_by_uploader(tmp_path, monkeypatch):
  # the comma policy keeps its caps; the zoo archive drops them
  assert Uploader.max_upload_sizes == MAX_UPLOAD_SIZES
  assert ZooUploader.max_upload_sizes == {}

  monkeypatch.setattr(pgp, 'encrypt_file', lambda src, out, recs, chunk_size=1 << 20: open(out, 'wb').write(open(src, 'rb').read()))
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader = make_uploader(tmp_path)
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'

  # this qlog is over the comma qlog cap, and the zoo uploader sends it anyway
  monkeypatch.setattr(Uploader, 'max_upload_sizes', {'qlog': 4})
  seg = write_segment(tmp_path, '00000017', {'qlog': b'0123456789'})
  ok = uploader.upload('qlog', '00000017--abcdefghij--1/qlog', str(seg / 'qlog'), 1, False)
  assert ok is True
  assert any(k.endswith('/00000017--abcdefghij--1/qlog.gpg') for k, _ in RecordingStub.seen)
  server.shutdown()


def test_epoch_skips_pre_enable_dirs(tmp_path):
  params = FakeParamsDict()
  params.put('RouteCount', '20')  # epoch records 20 (0x14): that dir is the first in
  params.put('BootCount', '3')
  uploader = make_uploader(tmp_path, params=params)
  assert uploader.epoch_route == 20 and uploader.epoch_boot == 3

  write_segment(tmp_path, '00000013', {'qlog': b'old'})
  write_segment(tmp_path, '00000014', {'qlog': b'first'})
  write_segment(tmp_path, '00000015', {'qlog': b'next'})
  (tmp_path / 'boot').mkdir()
  (tmp_path / 'boot' / '00000002--old.zst').write_bytes(b'oldboot')
  (tmp_path / 'boot' / '00000003--new.zst').write_bytes(b'newboot')

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  dirs = sorted(k.split('/')[0] for k in keys if not k.startswith(('boot/', 'crash/')))
  assert dirs == ['00000014--abcdefghij--1', '00000015--abcdefghij--1']
  boots = [k for k in keys if k.startswith('boot/')]
  assert boots == ['boot/00000003--new.zst']


def test_upload_flow_end_to_end(tmp_path, monkeypatch):
  uploader = make_uploader(tmp_path)
  # real PGP encryption needs a real recipient; encrypt through the module
  monkeypatch.setattr(pgp, 'encrypt_file', lambda src, out, recs, chunk_size=1 << 20: open(out, 'wb').write(b'GPG(' + open(src, 'rb').read() + b')'))

  # spin the recording stub and point the uploader at it
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'

  seg = write_segment(tmp_path, '00000016', {'qcamera.ts': b'video-bytes'})
  fn = seg / 'qcamera.ts'
  ok = uploader.upload('qcamera.ts', '00000016--abcdefghij--1/qcamera.ts', str(fn), 1, False)
  assert ok is True

  # object key shape and content through the whole pipeline
  path, body = RecordingStub.seen[-1]
  assert path == '/bucket/zooprefix/00000016--abcdefghij--1/qcamera.ts.gpg'
  assert body == b'GPG(video-bytes)'

  # tagged done with the fingerprint value; the comma xattr stays clear
  assert getxattr(str(fn), 'user.zoo_upload.s3') == uploader.upload_attr_value
  assert getxattr(str(fn), 'user.upload') is None

  # spool cleaned
  assert os.listdir(uploader.spool_dir) == []

  # status params
  assert uploader.params.get('ZooS3LastError') is None
  assert uploader.params.get('ZooS3UploadsDone') == b'1'
  server.shutdown()


def test_auth_failure_keeps_file_pending(tmp_path, monkeypatch):
  monkeypatch.setattr(pgp, 'encrypt_file', lambda src, out, recs, chunk_size=1 << 20: open(out, 'wb').write(open(src, 'rb').read()))
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader = make_uploader(tmp_path)
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'

  def fail_all(self):
    length = int(self.headers.get('Content-Length', 0))
    self.rfile.read(length)
    RecordingStub.seen.append((self.path, b''))
    self.send_response(403)
    self.send_header('Content-Type', 'application/xml')
    self.end_headers()
    self.wfile.write(b'<Error><Code>SignatureDoesNotMatch</Code></Error>')

  monkeypatch.setattr(RecordingStub, 'do_PUT', fail_all)

  seg = write_segment(tmp_path, '00000017', {'qlog': b'x'})
  fn = seg / 'qlog'
  ok = uploader.upload('qlog', '00000017--abcdefghij--1/qlog.zst', str(fn), 1, False)
  assert ok is False

  # NOT tagged: the file stays eligible, and the error is classified
  assert getxattr(str(fn), 'user.zoo_upload.s3') is None
  assert uploader.params.get('ZooS3LastError') == b'auth'
  assert uploader.params.get('ZooS3UploadsDone') is None
  server.shutdown()


def _fail_with_code(code: bytes, status: int):
  def handler(self):
    length = int(self.headers.get('Content-Length', 0))
    self.rfile.read(length)
    RecordingStub.seen.append((self.path, b''))
    self.send_response(status)
    self.send_header('Content-Type', 'application/xml')
    self.end_headers()
    self.wfile.write(b'<Error><Code>' + code + b'</Code></Error>')
  return handler


def test_permanent_failure_skips_file(tmp_path, monkeypatch):
  monkeypatch.setattr(pgp, 'encrypt_file', lambda src, out, recs, chunk_size=1 << 20: open(out, 'wb').write(open(src, 'rb').read()))
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader = make_uploader(tmp_path)
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'
  monkeypatch.setattr(RecordingStub, 'do_PUT', _fail_with_code(b'EntityTooLarge', 400))

  seg = write_segment(tmp_path, '00000021', {'qlog': b'x'})
  fn = seg / 'qlog'
  for _ in range(10):
    assert uploader.upload('qlog', '00000021--abcdefghij--1/qlog', str(fn), 1, False) is False

  # ten file-specific failures: tagged skipped, reported, and enumeration
  # moves on to the rest of the archive
  assert getxattr(str(fn), 'user.zoo_upload.s3') == uploader.skip_attr_value
  assert uploader.params.get('ZooS3LastError') == b'skipped_file'
  assert [k for _, k, _ in uploader.list_upload_files(metered=False)] == []
  server.shutdown()


def test_auth_outage_never_skips(tmp_path, monkeypatch):
  monkeypatch.setattr(pgp, 'encrypt_file', lambda src, out, recs, chunk_size=1 << 20: open(out, 'wb').write(open(src, 'rb').read()))
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader = make_uploader(tmp_path)
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'
  monkeypatch.setattr(RecordingStub, 'do_PUT', _fail_with_code(b'SignatureDoesNotMatch', 403))

  seg = write_segment(tmp_path, '00000021', {'qlog': b'x'})
  fn = seg / 'qlog'
  for _ in range(15):
    assert uploader.upload('qlog', '00000021--abcdefghij--1/qlog', str(fn), 1, False) is False

  # a credential problem fails every file equally: nothing gets skipped
  assert getxattr(str(fn), 'user.zoo_upload.s3') is None
  assert '00000021--abcdefghij--1/qlog' in [k for _, k, _ in uploader.list_upload_files(metered=False)]
  server.shutdown()


def test_transport_failure_never_skips(tmp_path, monkeypatch):
  import requests

  monkeypatch.setattr(pgp, 'encrypt_file', lambda src, out, recs, chunk_size=1 << 20: open(out, 'wb').write(open(src, 'rb').read()))
  from openpilot.system.loggerd import zoo_uploader as zoo_uploader_mod
  monkeypatch.setattr(zoo_uploader_mod.requests, 'put',
                      lambda *a, **k: (_ for _ in ()).throw(requests.exceptions.ConnectionError('no wire')))

  uploader = make_uploader(tmp_path)
  seg = write_segment(tmp_path, '00000021', {'qlog': b'x'})
  fn = seg / 'qlog'
  for _ in range(15):
    assert uploader.upload('qlog', '00000021--abcdefghij--1/qlog', str(fn), 1, False) is False

  # an outage must never skip a file either
  assert getxattr(str(fn), 'user.zoo_upload.s3') is None
  assert uploader._fail_counts == {}


def test_zero_size_upload_adds_no_stale_bytes(tmp_path):
  uploader = make_uploader(tmp_path)
  uploader._last_upload_bytes = 500  # stale from a previous file

  seg = write_segment(tmp_path, '00000022', {'qlog': b''})
  ok = uploader.upload('qlog', '00000022--abcdefghij--1/qlog', str(seg / 'qlog'), 1, False)
  assert ok is True
  assert uploader.bytes_since_prune == 0


def test_destination_change_epochs_only_after_success(tmp_path):
  params = FakeParamsDict()
  params.put('RouteCount', '20')
  params.put('BootCount', '3')
  uploader = make_uploader(tmp_path, params=params)
  write_segment(tmp_path, '00000016', {'qlog': b'x'})
  assert any('00000016' in k for _, k, _ in uploader.list_upload_files(metered=False))

  # zero successful uploads: a changed destination (typo fix) keeps the epoch
  uploader2 = make_uploader(tmp_path, params=params, bucket='fixed-bucket')
  assert uploader2.epoch_route == 20
  assert any('00000016' in k for _, k, _ in uploader2.list_upload_files(metered=False))

  # after successes, a destination change starts a new epoch at the counters now
  params.put('ZooS3UploadsDone', '5')
  params.put('RouteCount', '30')
  uploader3 = make_uploader(tmp_path, params=params, bucket='another-bucket')
  assert uploader3.epoch_route == 30
  assert not any('00000016' in k or '00000017' in k for _, k, _ in uploader3.list_upload_files(metered=False))


def test_comma_tag_does_not_suppress_zoo(tmp_path):
  uploader = make_uploader(tmp_path)
  seg = write_segment(tmp_path, '00000018', {'qlog': b'x'})
  fn = seg / 'qlog'
  setxattr(str(fn), 'user.upload', b'1')  # comma uploader already took it

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert '00000018--abcdefghij--1/qlog' in keys


def test_ring_off_by_default(tmp_path):
  # no budget param: the ring never talks to the bucket; an unparseable
  # value means off, not an outage. The default endpoint is unreachable
  # (port 9), so a stray bucket call would surface as a swallowed error.
  uploader = make_uploader(tmp_path)
  assert uploader.budget is None
  uploader.maybe_prune(force=True)

  bad_params = FakeParamsDict()
  bad_params.put('ZooS3ArchiveBudget', 'lots')
  bad = make_uploader(tmp_path, params=bad_params)
  assert bad.budget is None
  bad.maybe_prune(force=True)


def test_ring_prunes_whole_oldest_routes(tmp_path):
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '1300')
  uploader = make_uploader(tmp_path, params=params)
  assert uploader.budget == 1300

  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'
  RecordingStub.list_xml = list_result_xml([
    ('zooprefix/boot/00000002--bootid.zst.gpg', 50, '2026-01-01T00:00:00.000Z'),
    ('zooprefix/crash/crash1.gpg', 50, '2026-01-01T01:00:00.000Z'),
    ('zooprefix/00000001--abcdefghij/qlog.gpg', 300, '2026-01-02T00:00:00.000Z'),
    ('zooprefix/00000001--abcdefghij/qcamera.ts.gpg', 300, '2026-01-02T00:01:00.000Z'),
    ('zooprefix/00000002--abcdefghij/qlog.gpg', 400, '2026-01-03T00:00:00.000Z'),
    ('zooprefix/00000002--abcdefghij/qcamera.ts.gpg', 400, '2026-01-03T00:01:00.000Z'),
  ])
  RecordingStub.deletions = []
  try:
    uploader.maybe_prune(force=True)
  finally:
    server.shutdown()

  # total 1500, boot+crash 100 fixed: routes must shrink to 1200. The whole
  # oldest route (600) goes, the newer route and the protected boot/crash
  # objects stay
  assert sorted(RecordingStub.deletions) == [
    '/bucket/zooprefix/00000001--abcdefghij/qcamera.ts.gpg',
    '/bucket/zooprefix/00000001--abcdefghij/qlog.gpg',
  ]
  assert uploader.params.get('ZooS3LastPruneTime') is not None


def test_ring_trigger_gating(tmp_path, monkeypatch):
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '1G')
  uploader = make_uploader(tmp_path, params=params)
  calls = []
  monkeypatch.setattr(uploader, '_prune', lambda: calls.append(1) or 0)

  uploader.maybe_prune()
  assert calls == []

  uploader.bytes_since_prune = PRUNE_TRIGGER_BYTES
  uploader.maybe_prune()
  assert len(calls) == 1

  uploader.maybe_prune()
  assert len(calls) == 1  # the byte counter resets after a run


def test_ring_failure_is_tolerated(tmp_path, monkeypatch):
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '1G')
  uploader = make_uploader(tmp_path, params=params)
  monkeypatch.setattr(uploader, '_list_archive', lambda: (_ for _ in ()).throw(RuntimeError('list failed')))
  uploader.maybe_prune(force=True)  # a ring outage must not raise out of upload()


def _freeze_keep_markers(uploader, routes):
  # skip the bucket refresh; the marker set is the test input
  uploader._refresh_keep_routes = lambda: None
  uploader.keep_routes = set(routes)


def test_selected_scope_uploads_only_kept(tmp_path):
  uploader = make_uploader(tmp_path, scope='selected')
  _freeze_keep_markers(uploader, ['00000020--abcdefghij'])
  write_segment(tmp_path, '00000019', {'qlog': b'no', 'rlog': b'no'})
  write_segment(tmp_path, '00000020', {'qlog': b'yes', 'rlog': b'yes', 'qcamera.ts': b'v'})
  (tmp_path / 'crash').mkdir()
  (tmp_path / 'crash' / 'c1').write_bytes(b'c')

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  routes = sorted({k.split('/')[0] for k in keys if not k.startswith(('boot/', 'crash/'))})
  assert routes == ['00000020--abcdefghij--1']
  assert any(k.endswith('rlog') for k in keys)  # full files of a kept route
  assert any(k.startswith('crash/') for k in keys)


def test_selected_bypasses_epoch(tmp_path):
  params = FakeParamsDict()
  params.put('RouteCount', '20')  # epoch records 20: 00000013 is from before
  uploader = make_uploader(tmp_path, params=params, scope='selected')
  _freeze_keep_markers(uploader, ['00000013--abcdefghij'])
  write_segment(tmp_path, '00000013', {'qlog': b'old', 'rlog': b'old'})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert sorted(k.rsplit('/', 1)[-1] for k in keys) == ['qlog', 'rlog']


def test_selected_falls_back_to_silence_without_markers(tmp_path):
  # no markers anywhere (default endpoint unreachable): only crash and boot
  uploader = make_uploader(tmp_path, scope='selected')
  write_segment(tmp_path, '00000021', {'qlog': b'x'})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert keys == []


def test_keep_markers_read_from_bucket(tmp_path):
  uploader = make_uploader(tmp_path, scope='selected')
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'
  RecordingStub.list_xml = list_result_xml([
    ('zooprefix/keep/00000020--abcdefghij', 0, '2026-01-01T00:00:00.000Z'),
    ('zooprefix/keep/00000021--abcdefghij/', 0, '2026-01-01T00:00:00.000Z'),
  ])
  try:
    uploader._refresh_keep_routes()
  finally:
    server.shutdown()
  assert uploader.keep_routes == {'00000020--abcdefghij', '00000021--abcdefghij'}


def test_ring_spares_kept_and_fresh_routes(tmp_path):
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '1300')
  uploader = make_uploader(tmp_path, params=params)

  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'
  fresh = datetime.now(UTC).strftime('%Y-%m-%dT%H:%M:%S.000Z')
  RecordingStub.list_xml = list_result_xml([
    ('zooprefix/boot/00000002--bootid.zst.gpg', 50, '2026-01-01T00:00:00.000Z'),
    ('zooprefix/keep/00000001--abcdefghij', 0, '2026-01-01T00:00:00.000Z'),
    ('zooprefix/00000001--abcdefghij/qlog.gpg', 300, '2026-01-01T05:00:00.000Z'),  # kept
    ('zooprefix/00000002--abcdefghij/qlog.gpg', 300, fresh),  # too new to touch
    ('zooprefix/00000003--abcdefghij/qlog.gpg', 400, '2026-01-02T00:00:00.000Z'),
    ('zooprefix/00000003--abcdefghij/qcamera.ts.gpg', 400, '2026-01-02T00:01:00.000Z'),
    ('zooprefix/00000004--abcdefghij/qlog.gpg', 400, '2026-01-03T00:00:00.000Z'),
  ])
  RecordingStub.deletions = []
  try:
    uploader.maybe_prune(force=True)
  finally:
    server.shutdown()

  # routes total 1800, fixed 50: the ring must free 550. Route 1 is kept,
  # route 2 was just added: the whole oldest deletable route (3) goes, then
  # the ring stops under budget before touching kept or fresh data
  assert sorted(RecordingStub.deletions) == [
    '/bucket/zooprefix/00000003--abcdefghij/qcamera.ts.gpg',
    '/bucket/zooprefix/00000003--abcdefghij/qlog.gpg',
  ]
  assert not any('/keep/' in d for d in RecordingStub.deletions)


def test_normalize_route_request():
  dirs = ['0000000f--abcdef0123', '000001a3--c20ba54385']
  full = '000001a3--c20ba54385'
  assert normalize_route_request('1a3', dirs) == full
  assert normalize_route_request('01A3', dirs) == full
  assert normalize_route_request('1a3-c20ba54385', dirs) == full
  assert normalize_route_request(full, dirs) == full
  assert normalize_route_request('000001A3--C20BA54385', dirs) == full
  assert normalize_route_request('dongleid|000001a3--c20ba54385', dirs) == full
  assert normalize_route_request('dongleid/000001a3--c20ba54385', dirs) == full
  # a pasted segment directory still finds its route
  assert normalize_route_request('000001a3--c20ba54385--3', dirs) == full
  assert normalize_route_request('f', dirs) == '0000000f--abcdef0123'
  # wrong suffix on a real counter never matches
  assert normalize_route_request('1a3--deadbeef00', dirs) is None
  # unknown counters, non-hex text, and empty input never match
  assert normalize_route_request('2a3', dirs) is None
  assert normalize_route_request('15', dirs) is None
  assert normalize_route_request('zz', dirs) is None
  assert normalize_route_request('', dirs) is None


def test_upload_request_helpers():
  params = FakeParamsDict()
  assert add_upload_request(params, '000001a3--c20ba54385') is True
  assert add_upload_request(params, '000001a3--c20ba54385') is False
  assert load_upload_requests(params) == [{'route': '000001a3--c20ba54385', 'uploaded_any': False}]
  # corrupt JSON falls back to an empty list, never raises
  params.put('ZooS3UploadRoutes', 'not json')
  assert load_upload_requests(params) == []


def test_current_drive_request():
  # zoo S3 off queues nothing
  params = FakeParamsDict()
  params.put('CurrentRoute', '000001a3--c20ba54385')
  assert add_current_drive_request(params) is None
  assert load_upload_requests(params) == []

  # no active route queues nothing
  params = FakeParamsDict()
  params.put('ZooS3Enabled', '1')
  assert add_current_drive_request(params) is None
  assert load_upload_requests(params) == []

  # active route queues once; a second swipe is idempotent
  params.put('CurrentRoute', '000001a3--c20ba54385')
  assert add_current_drive_request(params) == '000001a3--c20ba54385'
  assert add_current_drive_request(params) == '000001a3--c20ba54385'
  assert load_upload_requests(params) == [{'route': '000001a3--c20ba54385', 'uploaded_any': False}]


def test_latest_route_dir(tmp_path):
  (tmp_path / 'boot').mkdir()
  write_segment(tmp_path, '00000002', {'qlog': b'x'})
  write_segment(tmp_path, '00000010', {'qlog': b'x'})
  assert latest_route_dir(str(tmp_path)) == '00000010--abcdefghij'


def test_route_of_groups_segments(tmp_path):
  # one drive, two segments on disk: the route identity covers both
  write_segment(tmp_path, '0000002b', {'qlog': b'one'}, part=1)
  write_segment(tmp_path, '0000002b', {'qlog': b'two'}, part=2)
  assert list_route_dirs(str(tmp_path)) == ['0000002b--abcdefghij']
  assert latest_route_dir(str(tmp_path)) == '0000002b--abcdefghij'


def test_bookmark_covers_every_segment(tmp_path):
  # the on-road bookmark queues the route name the logger reports; the
  # upload must reach every segment directory of that route
  params = FakeParamsDict()
  params.put('ZooS3Enabled', '1')
  params.put('CurrentRoute', '0000002b--abcdefghij')
  write_segment(tmp_path, '0000002b', {'qlog': b'one'}, part=1)
  write_segment(tmp_path, '0000002b', {'qcamera.ts': b'two'}, part=2)

  assert add_current_drive_request(params) == '0000002b--abcdefghij'
  uploader = make_uploader(tmp_path, params=params)
  keys = sorted(k for _, k, _ in uploader.list_upload_files(metered=False))
  assert keys == ['0000002b--abcdefghij--1/qlog', '0000002b--abcdefghij--2/qcamera.ts']


def test_window_counts_whole_routes(tmp_path):
  # segments of one route count together in the window: with a 250-byte
  # budget, the 100-byte newest route fits and the 200-byte older route
  # stays out — a per-segment mistake would let one segment slip in
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '250')
  uploader = make_uploader(tmp_path, params=params)
  write_segment(tmp_path, '0000001a', {'qlog': b'a' * 100}, part=1)
  write_segment(tmp_path, '0000001a', {'qlog': b'a' * 100}, part=2)
  write_segment(tmp_path, '0000001b', {'qlog': b'b' * 100})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert sorted({k.split('/')[0] for k in keys}) == ['0000001b--abcdefghij--1']


def test_window_uploads_newest_budget_worth(tmp_path):
  # budget 250 bytes, scope counts 100 per route: the two newest routes fit,
  # the third would push the archive over and stays on the device. The epoch
  # (RouteCount 30) is above every counter here — the window replaces it.
  params = FakeParamsDict()
  params.put('RouteCount', '30')
  params.put('ZooS3ArchiveBudget', '250')
  uploader = make_uploader(tmp_path, params=params)
  write_segment(tmp_path, '00000013', {'qlog': b'a' * 100})
  write_segment(tmp_path, '00000014', {'qlog': b'b' * 100})
  write_segment(tmp_path, '00000015', {'qlog': b'c' * 100})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  dirs = sorted({k.split('/')[0] for k in keys})
  assert dirs == ['00000014--abcdefghij--1', '00000015--abcdefghij--1']


def test_window_current_drive_always_in(tmp_path):
  # a route larger than the whole budget still uploads when it is the
  # newest: the current drive must make it to the archive
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '50')
  uploader = make_uploader(tmp_path, params=params)
  write_segment(tmp_path, '0000002a', {'qlog': b'x' * 100})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert [k.split('/')[0] for k in keys] == ['0000002a--abcdefghij--1']


def test_window_slides_as_routes_record(tmp_path):
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '200')
  uploader = make_uploader(tmp_path, params=params)
  write_segment(tmp_path, '0000001a', {'qlog': b'a' * 100})
  write_segment(tmp_path, '0000001b', {'qlog': b'b' * 100})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert sorted({k.split('/')[0] for k in keys}) == ['0000001a--abcdefghij--1', '0000001b--abcdefghij--1']

  # a new route pushes the oldest one out of the window: it stays on the
  # device, uploaded files are never re-uploaded or un-uploaded
  write_segment(tmp_path, '0000001c', {'qlog': b'c' * 100})
  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert sorted({k.split('/')[0] for k in keys}) == ['0000001b--abcdefghij--1', '0000001c--abcdefghij--1']


def test_window_counts_scope_masked(tmp_path):
  # minimal scope uploads qlogs only, so the big rlog is invisible to the
  # window and both routes fit; under full scope the rlog counts and the
  # older route falls out
  files = {'qlog': b'q' * 100, 'rlog': b'r' * 10000}
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '250')

  uploader = make_uploader(tmp_path, params=params, scope='minimal')
  write_segment(tmp_path, '0000001a', files)
  write_segment(tmp_path, '0000001b', {'qlog': b'b' * 100})
  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert sorted({k.split('/')[0] for k in keys}) == ['0000001a--abcdefghij--1', '0000001b--abcdefghij--1']

  uploader = make_uploader(tmp_path, params=params, scope='full')
  write_segment(tmp_path, '0000001a', files)
  write_segment(tmp_path, '0000001b', {'qlog': b'b' * 100})
  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert [k.split('/')[0] for k in keys] == ['0000001b--abcdefghij--1']


def test_window_requests_and_keeps_bypass(tmp_path):
  # a requested route uploads whatever the window says; the same holds for
  # a kept route under selected scope
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '50')
  params.put('RouteCount', '30')
  uploader = make_uploader(tmp_path, params=params)
  add_upload_request(params, '00000010--abcdefghij')
  write_segment(tmp_path, '00000010', {'qlog': b'old' * 100, 'qcamera.ts': b'v'})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert sorted(k.rsplit('/', 1)[-1] for k in keys if k.startswith('00000010')) == ['qcamera.ts', 'qlog']

  uploader = make_uploader(tmp_path, params=params, scope='selected')
  _freeze_keep_markers(uploader, ['00000011--abcdefghij'])
  write_segment(tmp_path, '00000011', {'qlog': b'old' * 100, 'rlog': b'r'})
  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert sorted(k.rsplit('/', 1)[-1] for k in keys if k.startswith('00000011')) == ['qlog', 'rlog']


def test_window_leaves_boot_epoch_gated(tmp_path):
  # the window is route-only: boot files keep their enable-epoch gate even
  # when a budget is set
  params = FakeParamsDict()
  params.put('RouteCount', '30')
  params.put('BootCount', '2')
  params.put('ZooS3ArchiveBudget', '250')
  uploader = make_uploader(tmp_path, params=params)
  write_segment(tmp_path, '00000010', {'qlog': b'x' * 10})
  (tmp_path / 'boot').mkdir()
  (tmp_path / 'boot' / '00000001--old.zst').write_bytes(b'oldboot')
  (tmp_path / 'boot' / '00000002--new.zst').write_bytes(b'newboot')

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  boots = [k for k in keys if k.startswith('boot/')]
  assert boots == ['boot/00000002--new.zst']
  assert any(k.startswith('00000010--abcdefghij--1/') for k in keys)


def test_window_unset_keeps_epoch(tmp_path):
  # no budget: today's behavior — the enable epoch is the only gate
  params = FakeParamsDict()
  params.put('RouteCount', '20')
  uploader = make_uploader(tmp_path, params=params)
  write_segment(tmp_path, '00000013', {'qlog': b'old'})
  write_segment(tmp_path, '00000014', {'qlog': b'new'})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  assert sorted({k.split('/')[0] for k in keys}) == ['00000014--abcdefghij--1']


def test_upload_request_bypasses_scope_and_epoch(tmp_path):
  params = FakeParamsDict()
  params.put('RouteCount', '20')  # epoch records 20: route 13 predates it
  uploader = make_uploader(tmp_path, params=params, scope='minimal', contents='everything')
  add_upload_request(params, '00000013--abcdefghij')
  write_segment(tmp_path, '00000013', {'qlog': b'x', 'rlog': b'r', 'fcamera.hevc': b'f'})
  write_segment(tmp_path, '00000019', {'rlog': b'other'})

  keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
  # minimal scope leaves 19 alone, but the request pulls the whole drive in
  assert '00000019--abcdefghij--1/rlog' not in keys
  assert sorted(k.rsplit('/', 1)[-1] for k in keys if k.startswith('00000013')) == ['fcamera.hevc', 'qlog', 'rlog']


def test_upload_request_contents_mask(tmp_path):
  files = {'qlog': b'x', 'qcamera.ts': b'v', 'rlog': b'r', 'fcamera.hevc': b'f', 'ecamera.hevc': b'e', 'dcamera.hevc': b'd'}
  cases = (
    ('default', ['qcamera.ts', 'qlog']),
    ('road', ['fcamera.hevc', 'rlog']),
    ('everything', sorted(files)),
  )
  for contents, expected in cases:
    params = FakeParamsDict()
    params.put('ZooS3UploadContents', contents)
    uploader = make_uploader(tmp_path, params=params, scope='minimal', contents=contents)
    add_upload_request(params, '00000013--abcdefghij')
    write_segment(tmp_path, '00000013', files)

    keys = [key for _, key, _ in uploader.list_upload_files(metered=False)]
    names = sorted(k.rsplit('/', 1)[-1] for k in keys if k.startswith('00000013'))
    assert names == expected, contents


def test_upload_request_fulfills_and_writes_marker(tmp_path, monkeypatch):
  monkeypatch.setattr(pgp, 'encrypt_file', lambda src, out, recs, chunk_size=1 << 20: open(out, 'wb').write(b'G(' + open(src, 'rb').read() + b')'))
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader = make_uploader(tmp_path)
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'
  add_upload_request(uploader.params, '00000030--abcdefghij')
  write_segment(tmp_path, '00000030', {'qlog': b'x', 'rlog': b'y'})

  keys = sorted(k for _, k, _ in uploader.list_upload_files(metered=False))
  for key in keys:
    assert uploader.upload(key.rsplit('/', 1)[-1], key, os.path.join(str(tmp_path), key), 1, False) is True

  # fully uploaded: the pending entry clears and the marker lands in keep/
  assert load_upload_requests(uploader.params) == []
  marker_puts = [path for path, _ in RecordingStub.seen if '/keep/' in path]
  assert marker_puts == ['/bucket/zooprefix/keep/00000030--abcdefghij']
  server.shutdown()


def test_upload_request_abandoned_when_route_vanishes(tmp_path):
  uploader = make_uploader(tmp_path)
  add_upload_request(uploader.params, '00000040--abcdefghij')

  RecordingStub.seen = []
  uploader.service_upload_requests()
  # deleted before a single file uploaded: dropped, no marker ever written
  assert load_upload_requests(uploader.params) == []
  assert not any('/keep/' in path for path, _ in RecordingStub.seen)


def test_ring_spares_pending_requests(tmp_path):
  params = FakeParamsDict()
  params.put('ZooS3ArchiveBudget', '1300')
  uploader = make_uploader(tmp_path, params=params)
  add_upload_request(params, '00000001--abcdefghij')

  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'
  RecordingStub.list_xml = list_result_xml([
    ('zooprefix/boot/00000002--bootid.zst.gpg', 50, '2026-01-01T00:00:00.000Z'),
    ('zooprefix/00000001--abcdefghij/qlog.gpg', 300, '2026-01-01T00:00:00.000Z'),  # pending
    ('zooprefix/00000002--abcdefghij/qlog.gpg', 300, '2026-01-02T00:00:00.000Z'),
    ('zooprefix/00000003--abcdefghij/qlog.gpg', 400, '2026-01-03T00:00:00.000Z'),
    ('zooprefix/00000004--abcdefghij/qlog.gpg', 400, '2026-01-04T00:00:00.000Z'),
  ])
  RecordingStub.deletions = []
  try:
    uploader.maybe_prune(force=True)
  finally:
    server.shutdown()

  # routes total 1400, target 1250: the pending route is oldest but is
  # protected, so the ring takes the next route instead and stops under
  # budget
  assert RecordingStub.deletions == ['/bucket/zooprefix/00000002--abcdefghij/qlog.gpg']


def test_service_keeps_concurrent_additions(tmp_path, monkeypatch):
  # a button press landing between the daemon's read and its write must
  # survive the sweep
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader = make_uploader(tmp_path)
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'

  from openpilot.system.loggerd import zoo_uploader as zoo_uploader_mod
  reads = [
    [{'route': '00000030--abcdefghij', 'uploaded_any': True}],  # the sweep's read: finished
    [{'route': '00000030--abcdefghij', 'uploaded_any': True},   # the re-read: a press landed
     {'route': '00000031--abcdefghij', 'uploaded_any': False}],
  ]
  monkeypatch.setattr(zoo_uploader_mod, 'load_upload_requests', lambda params: reads.pop(0))

  uploader.service_upload_requests()
  assert load_upload_requests(uploader.params) == [{'route': '00000031--abcdefghij', 'uploaded_any': False}]
  server.shutdown()


def test_encrypted_uncompressed_file_needs_one_spool_file(tmp_path, monkeypatch):
  # flash accounting: encrypting fcamera.hevc spills only the ciphertext
  # (one .gpg); the compress path still materializes both spool files
  monkeypatch.setattr(pgp, 'encrypt_file', lambda src, out, recs, chunk_size=1 << 20: open(out, 'wb').write(b'G(' + open(src, 'rb').read() + b')'))
  from openpilot.system.loggerd import zoo_uploader as zoo_uploader_mod
  calls = []
  real_mkstemp = zoo_uploader_mod.tempfile.mkstemp

  def counting_mkstemp(*a, **k):
    calls.append(k.get('suffix'))
    return real_mkstemp(*a, **k)

  monkeypatch.setattr(zoo_uploader_mod.tempfile, 'mkstemp', counting_mkstemp)
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader = make_uploader(tmp_path)  # encrypt=True
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'

  seg = write_segment(tmp_path, '00000016', {'fcamera.hevc': b'video'})
  assert uploader.upload('fcamera.hevc', '00000016--abcdefghij--1/fcamera.hevc', str(seg / 'fcamera.hevc'), 1, False) is True
  assert calls == ['.gpg']

  calls.clear()
  seg = write_segment(tmp_path, '00000017', {'qlog': b'x'})
  assert uploader.upload('qlog', '00000017--abcdefghij--1/qlog.zst', str(seg / 'qlog'), 1, False) is True
  assert calls == ['.plain', '.gpg']
  server.shutdown()


def test_plaintext_upload_stores_readable_objects(tmp_path):
  uploader = make_uploader(tmp_path, encrypt=False)
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'

  seg = write_segment(tmp_path, '00000016', {'qcamera.ts': b'video-bytes'})
  fn = seg / 'qcamera.ts'
  assert uploader.upload('qcamera.ts', '00000016--abcdefghij--1/qcamera.ts', str(fn), 1, False) is True

  # no .gpg suffix and the body is the raw file bytes
  path, body = RecordingStub.seen[-1]
  assert path == '/bucket/zooprefix/00000016--abcdefghij--1/qcamera.ts'
  assert body == b'video-bytes'
  assert getxattr(str(fn), 'user.zoo_upload.s3') == uploader.upload_attr_value
  server.shutdown()


def test_plain_upload_never_touches_the_spool(tmp_path, monkeypatch):
  # a plain, already-compressed file takes the direct path: any attempt
  # to spool it turns into a failure, so a passing upload proves the
  # original streamed straight out
  from openpilot.system.loggerd import zoo_uploader as zoo_uploader_mod
  monkeypatch.setattr(zoo_uploader_mod.tempfile, 'mkstemp',
                      lambda *a, **k: (_ for _ in ()).throw(AssertionError('spool used')))

  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), RecordingStub)
  threading.Thread(target=server.serve_forever, daemon=True).start()
  uploader = make_uploader(tmp_path, encrypt=False)
  uploader.s3.endpoint = f'http://127.0.0.1:{server.server_address[1]}'

  seg = write_segment(tmp_path, '00000016', {'qcamera.ts': b'video-bytes'})
  ok = uploader.upload('qcamera.ts', '00000016--abcdefghij--1/qcamera.ts', str(seg / 'qcamera.ts'), 1, False)
  assert ok is True
  assert RecordingStub.seen[-1] == ('/bucket/zooprefix/00000016--abcdefghij--1/qcamera.ts', b'video-bytes')
  server.shutdown()


def test_config_encryption_needs_key(monkeypatch):
  monkeypatch.setattr(pgp, 'load_recipients', lambda armored, strict_expiry=True: ['fake-recipient'])

  def base_params():
    params = FakeParamsDict()
    for key, value in (('ZooS3Endpoint', 'https://s3.example.com'), ('ZooS3Bucket', 'b'),
                       ('ZooS3AccessKeyId', 'k'), ('ZooS3SecretAccessKey', 's')):
      params.put(key, value)
    return params

  # encryption on without a key: refuse to start, never silently go plaintext
  params = base_params()
  params.put('ZooS3Encryption', '1')
  assert zoo_config_from_params(params) is None
  assert params.get('ZooS3LastError') == b'missing_key'

  # key supplied: config carries the encryption flag and recipients
  params = base_params()
  params.put('ZooS3Encryption', '1')
  params.put('ZooS3PublicKey', '-----BEGIN PGP PUBLIC KEY BLOCK-----')
  config = zoo_config_from_params(params)
  assert config is not None and config[4] is True

  # encryption off: the key is not needed and is not loaded
  params = base_params()
  config = zoo_config_from_params(params)
  assert config is not None and config[4] is False and config[5] == []
