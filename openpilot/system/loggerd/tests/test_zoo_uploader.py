import http.server
import os
import sys
import threading
import types
from datetime import datetime, timezone


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
from openpilot.common.zoo.s3sig import S3Config
from openpilot.system.loggerd.zoo_uploader import ZooUploader, PRUNE_TRIGGER_BYTES
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


def make_uploader(tmp_path, params=None, scope='minimal', status_map=None, bucket='bucket'):
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
  return ZooUploader(str(tmp_path), s3, 'zooprefix', scope, [recipient], params)


def write_segment(root, counter_hex, files):
  seg = root / f'{counter_hex}--abcdefghij'
  seg.mkdir(parents=True, exist_ok=True)
  for name, data in files.items():
    (seg / name).write_bytes(data)
  return seg


def list_result_xml(entries):
  contents = ''.join(
    f'<Contents><Key>{k}</Key><Size>{s}</Size><LastModified>{d}</LastModified></Contents>'
    for k, s, d in entries)
  return f'<?xml version="1.0"?><ListBucketResult xmlns="http://s3.amazonaws.com/doc/2006-03-01/"><IsTruncated>false</IsTruncated>{contents}</ListBucketResult>'.encode()


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
  assert dirs == ['00000014--abcdefghij', '00000015--abcdefghij']
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
  ok = uploader.upload('qcamera.ts', '00000016--abcdefghij/qcamera.ts', str(fn), 1, False)
  assert ok is True

  # object key shape and content through the whole pipeline
  path, body = RecordingStub.seen[-1]
  assert path == '/bucket/zooprefix/00000016--abcdefghij/qcamera.ts.gpg'
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
  ok = uploader.upload('qlog', '00000017--abcdefghij/qlog.zst', str(fn), 1, False)
  assert ok is False

  # NOT tagged: the file stays eligible, and the error is classified
  assert getxattr(str(fn), 'user.zoo_upload.s3') is None
  assert uploader.params.get('ZooS3LastError') == b'auth'
  assert uploader.params.get('ZooS3UploadsDone') is None
  server.shutdown()


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
  assert '00000018--abcdefghij/qlog' in keys


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
  assert routes == ['00000020--abcdefghij']
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
  fresh = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.000Z')
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
