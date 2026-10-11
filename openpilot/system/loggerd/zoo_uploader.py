#!/usr/bin/env python3
"""Private S3 archive uploader for the zoo backend.

Uploads the same files the comma uploader sees, encrypted to the user's
PGP public keys, to the user's own S3-compatible bucket. Tracks completion
with its own xattr so comma uploads and zoo uploads never suppress each
other, and never marks a file done on an authentication failure.

Scope 'minimal' uploads qlogs, the compressed camera, and crash/boot logs;
'full' uploads everything; 'selected' uploads only routes with a keep
marker under keep/<route>, plus crash/boot logs. Keep markers also spare
their routes from the archive ring.
"""

import os
import hashlib
import random
import re
import shutil
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime

import requests

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.utils import get_upload_stream
from openpilot.common.realtime import set_core_affinity
from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.uploader import Uploader, clear_locks
from openpilot.common.zoo import pgp
from openpilot.common.zoo.s3sig import S3Config, signed_put_file, signed_request, parse_budget

NetworkType = log.DeviceState.NetworkType

SCOPE_MINIMAL_FILES = {'qlog', 'qlog.zst', 'qcamera.ts'}
SPOOL_SUBDIR = '.zoo-spool'
S3_NS = {'s3': 'http://s3.amazonaws.com/doc/2006-03-01/'}

# keep markers: an object keep/<route> (a provider-console "folder" inside
# keep/ works too) selects that route. Selected scope uploads only kept
# routes, and the ring never deletes one.
KEEP_MARKER_DIR = 'keep'
KEEP_REFRESH_SECONDS = 300

# the ring never deletes a route whose newest object is younger than this,
# so it cannot fight the uploader for the same files
PRUNE_MIN_AGE_SECONDS = 24 * 60 * 60

# one list+sweep per GiB uploaded keeps ring traffic negligible next to the
# uploads it meters
PRUNE_TRIGGER_BYTES = 1 << 30

# S3 <Code> to a short, sanitized reason; never put the response body or the
# signed URL into params or logs
ERROR_CLASSES = {
  'RequestTimeTooSkewed': 'clock_skew',
  'SignatureDoesNotMatch': 'auth',
  'InvalidAccessKeyId': 'auth',
  'AccessDenied': 'auth',
  'Unauthorized': 'auth',
  'AuthorizationHeaderMalformed': 'endpoint',
  'PermanentRedirect': 'endpoint',
  'NoSuchBucket': 'endpoint',
  'SlowDown': 'slowdown',
}


class ZooUploader(Uploader):
  upload_attr_name = 'user.zoo_upload.s3'
  success_codes = (200, 201)

  def __init__(self, root: str, s3: S3Config, prefix: str, scope: str, recipients: list, params: Params):
    self.api = None  # no backend API for this destination
    self.dongle_id = prefix
    self.root = root
    self.params = params
    self.s3 = s3
    self.scope = scope
    self.recipients = recipients
    self.last_filename = ""
    self.immediate_folders = ["crash/", "boot/"]
    self.immediate_priority = {"qlog": 0, "qlog.zst": 0, "qcamera.ts": 1}

    # the xattr value carries the destination fingerprint: files tagged for
    # another bucket or prefix are pending again, and a requeue is one
    # xattr-clearing pass
    self.upload_attr_value = self._destination_fingerprint().encode()

    self.spool_dir = os.path.join(root, SPOOL_SUBDIR)
    os.makedirs(self.spool_dir, exist_ok=True)
    for stale in os.listdir(self.spool_dir):
      # spooled ciphertext is never reused across restarts, keys, or epochs
      try:
        os.unlink(os.path.join(self.spool_dir, stale))
      except OSError:
        pass

    # archive ring: when a budget is set, oldest objects are pruned until the
    # bucket fits it; an unparseable value means the ring stays off
    budget_text = (self.params.get('ZooS3ArchiveBudget') or b'').decode()
    self.budget = parse_budget(budget_text)
    if budget_text and self.budget is None:
      cloudlog.warning('zoo archive ring: unparseable ZooS3ArchiveBudget, ring off')
    self.bytes_since_prune = 0

    # route selection; filled from keep markers, re-read at most every
    # KEEP_REFRESH_SECONDS
    self.keep_routes: set[str] = set()
    self._keep_checked_at = 0.0

    self._init_epoch()

  def _destination_fingerprint(self) -> str:
    material = '|'.join((self.s3.endpoint, self.s3.bucket, self.dongle_id))
    return hashlib.sha256(material.encode()).hexdigest()[:16]

  def _init_epoch(self) -> None:
    """Clock-free start-from-now bookkeeping.

    A destination change starts a new epoch only after at least one
    successful upload to the old destination; a fixed typo (zero uploads)
    keeps the epoch so the pending backlog flows to the corrected bucket.
    """
    fp = self._destination_fingerprint()
    stored_fp = self.params.get('ZooS3DestinationFp')
    done = self._counter('ZooS3UploadsDone')

    if stored_fp is not None and stored_fp.decode(errors='ignore') != fp and done > 0:
      self._set_epoch()  # new destination, old one has data
      self.params.put('ZooS3UploadsDone', '0')
    elif stored_fp is None or self.params.get('ZooS3EpochRouteCount') is None:
      self._set_epoch()  # first enable

    self.params.put('ZooS3DestinationFp', fp)
    self.epoch_route = self._counter('ZooS3EpochRouteCount')
    self.epoch_boot = self._counter('ZooS3EpochBootCount')

  def _set_epoch(self) -> None:
    self.params.put('ZooS3EpochRouteCount', (self.params.get('RouteCount') or b'0').decode())
    self.params.put('ZooS3EpochBootCount', (self.params.get('BootCount') or b'0').decode())

  def _counter(self, key: str) -> int:
    try:
      return int(self.params.get(key) or b'0')
    except ValueError:
      return 0

  def list_upload_files(self, metered: bool):
    self._refresh_keep_routes()
    for name, key, fn in super().list_upload_files(metered):
      if not self._in_scope(key):
        continue
      # a selected scope means the user picked the routes, so the enable
      # moment does not gate them
      if self.scope == 'selected' or self._after_epoch(key):
        yield name, key, fn

  def _in_scope(self, key: str) -> bool:
    top = key.split('/', 1)[0]
    if top in ('crash', 'boot'):
      return True
    if self.scope == 'full':
      return True
    if self.scope == 'selected':
      return self._route_of(key) in self.keep_routes
    return key.rsplit('/', 1)[-1] in SCOPE_MINIMAL_FILES

  @staticmethod
  def _route_of(key: str) -> str:
    return key.split('/', 1)[0]

  def _refresh_keep_routes(self) -> None:
    if self.scope != 'selected':
      return  # other scopes never consult the marker set
    now = time.monotonic()
    if now - self._keep_checked_at < KEEP_REFRESH_SECONDS:
      return
    self._keep_checked_at = now
    try:
      self.keep_routes = self._list_keep_markers()
    except Exception:
      # without markers, Selected scope holds everything but crash and boot
      cloudlog.exception('zoo_keep_marker_refresh_failed')

  def _after_epoch(self, key: str) -> bool:
    parts = key.split('/')
    if parts[0] == 'crash':
      return True  # crash names carry no counter; small, always in scope
    ident = parts[1] if parts[0] == 'boot' else parts[0]
    counter_text = ident.split('--', 1)[0]
    try:
      counter = int(counter_text, 16)
    except ValueError:
      return True  # not a counter-named directory
    epoch = self.epoch_boot if parts[0] == 'boot' else self.epoch_route
    return counter >= epoch

  def do_upload(self, key: str, fn: str):
    compress = key.endswith('.zst') and not fn.endswith('.zst')
    plain_fd, plain_path = tempfile.mkstemp(dir=self.spool_dir, suffix='.plain')
    gpg_fd, gpg_path = tempfile.mkstemp(dir=self.spool_dir, suffix='.gpg')
    os.close(plain_fd)
    os.close(gpg_fd)
    try:
      stream, _ = get_upload_stream(fn, compress)
      try:
        with open(plain_path, 'wb') as out:
          shutil.copyfileobj(stream, out)
      finally:
        stream.close()

      pgp.encrypt_file(plain_path, gpg_path, self.recipients)
      os.unlink(plain_path)

      object_key = f'{self.dongle_id}/{key}.gpg'
      url, headers = signed_put_file(self.s3, object_key, gpg_path)
      self._last_upload_bytes = os.path.getsize(gpg_path)
      with open(gpg_path, 'rb') as body:
        response = requests.put(url, headers=headers, data=body, timeout=30)
      self._last_response = response
      return response
    finally:
      for path in (plain_path, gpg_path):
        try:
          os.unlink(path)
        except OSError:
          pass

  def upload(self, name: str, key: str, fn: str, network_type: int, metered: bool) -> bool | None:
    result = super().upload(name, key, fn, network_type, metered)
    if result:
      # a wall-clock stamp is what the status param means
      self.params.put('ZooS3LastUploadTime', str(int(time.time())))  # noqa: TID251
      self.params.put('ZooS3UploadsDone', str(self._counter('ZooS3UploadsDone') + 1))
      self.bytes_since_prune += getattr(self, '_last_upload_bytes', 0)
      self.maybe_prune()
    else:
      self._record_error(key)
    return result

  def maybe_prune(self, force: bool = False) -> None:
    """Keep the remote archive under the budget, oldest routes first.

    Never raises: a ring failure must not take uploads down with it. The
    next trigger retries.
    """
    if self.budget is None:
      return
    if not force and self.bytes_since_prune < PRUNE_TRIGGER_BYTES:
      return
    self.bytes_since_prune = 0
    try:
      deleted = self._prune()
      if deleted:
        self.params.put('ZooS3LastPruneTime', str(int(time.time())))  # noqa: TID251
        cloudlog.event('zoo_archive_pruned', deleted=deleted)
    except Exception:
      cloudlog.exception('zoo_archive_prune_failed')

  def _prune(self) -> int:
    """Delete whole oldest routes until the archive fits the budget.

    boot/ and crash/ objects, keep-marked routes, and routes with objects
    younger than a day are protected: a crash payload cannot be
    regenerated, a kept route was picked on purpose, and the ring must
    not fight the uploader for the same files. Protection still counts
    toward the budget it occupies; when protected data alone is over the
    budget, the ring deletes nothing further.
    """
    objects = self._list_archive()
    keep_prefix = f'{self.dongle_id}/{KEEP_MARKER_DIR}/'
    protected: set[str] = set()
    routes: dict[str, list[tuple[int, str, str]]] = {}
    fixed = 0
    route_total = 0
    now = time.time()
    for key, size, lastmod in objects:
      group = key.split('/', 2)[1]  # the component after the prefix
      if group == KEEP_MARKER_DIR:
        fixed += size
        name = key.removeprefix(keep_prefix).split('/', 1)[0]
        if name:
          protected.add(name)
      elif group in ('boot', 'crash'):
        fixed += size
      else:
        route_total += size
        routes.setdefault(group, []).append((size, lastmod, key))

    target = self.budget - fixed
    if route_total <= target:
      return 0
    deleted = 0
    for group in sorted(routes, key=lambda g: min(m for _, m, _ in routes[g])):
      if route_total <= target:
        break
      if group in protected or self._route_is_fresh(routes[group], now):
        continue
      for size, _, key in routes[group]:
        url, headers = signed_request(self.s3, 'DELETE', key)
        response = requests.delete(url, headers=headers, timeout=30)
        if response.status_code not in (200, 204):
          raise RuntimeError(f'archive prune delete failed: HTTP {response.status_code}')
        route_total -= size
        deleted += 1
    return deleted

  @staticmethod
  def _route_is_fresh(entries: list[tuple[int, str, str]], now: float) -> bool:
    for _, lastmod, _ in entries:
      stamp = _parse_last_modified(lastmod)
      if stamp is not None and now - stamp < PRUNE_MIN_AGE_SECONDS:
        return True
    return False

  def _list_archive(self) -> list[tuple[str, int, str]]:
    """All (key, size, last-modified) under the archive prefix."""
    return self._list_objects(f'{self.dongle_id}/')

  def _list_keep_markers(self) -> set[str]:
    """Route names with a keep marker under keep/."""
    routes = set()
    keep_prefix = f'{self.dongle_id}/{KEEP_MARKER_DIR}/'
    for key, _, _ in self._list_objects(keep_prefix):
      name = key.removeprefix(keep_prefix).split('/', 1)[0].strip()
      if name:
        routes.add(name)
    return routes

  def _list_objects(self, prefix: str) -> list[tuple[str, int, str]]:
    """All (key, size, last-modified) under one prefix."""
    objects = []
    token = ''
    while True:
      query = {'list-type': '2', 'prefix': prefix}
      if token:
        query['continuation-token'] = token
      url, headers = signed_request(self.s3, 'GET', '', query=query)
      response = requests.get(url, headers=headers, timeout=30)
      response.raise_for_status()
      root = ET.fromstring(response.content)
      for contents in root.findall('s3:Contents', S3_NS):
        objects.append((
          contents.findtext('s3:Key', '', S3_NS),
          int(contents.findtext('s3:Size', '0', S3_NS)),
          contents.findtext('s3:LastModified', '', S3_NS),
        ))
      token = root.findtext('s3:NextContinuationToken', '', S3_NS)
      if not token:
        return objects

  def _record_error(self, key: str) -> None:
    reason = 'upload_failed'
    try:
      stat = getattr(self, '_last_response', None)
      if stat is not None and getattr(stat, 'text', ''):
        code = re.search(r'<Code>([^<]+)</Code>', stat.text[:4096])
        if code:
          reason = ERROR_CLASSES.get(code.group(1), code.group(1))
    except Exception:
      cloudlog.exception('zoo_upload_error_classify')
    self.params.put('ZooS3LastError', reason)
    cloudlog.event('zoo_upload_failed', key=key, reason=reason)


def _parse_last_modified(text: str) -> float | None:
  """S3 LastModified (ISO 8601) to epoch seconds; None when unparseable."""
  try:
    return datetime.fromisoformat(text.replace('Z', '+00:00')).timestamp()
  except ValueError:
    return None


def zoo_config_from_params(params: Params) -> tuple[S3Config, str, str, list] | None:
  endpoint = params.get('ZooS3Endpoint')
  bucket = params.get('ZooS3Bucket')
  key_id = params.get('ZooS3AccessKeyId')
  secret = params.get('ZooS3SecretAccessKey')
  public_key = params.get('ZooS3PublicKey')
  if not all((endpoint, bucket, key_id, secret, public_key)):
    return None

  s3 = S3Config(
    endpoint=endpoint.decode(),
    region=(params.get('ZooS3Region') or b'us-east-1').decode(),
    bucket=bucket.decode(),
    access_key_id=key_id.decode(),
    secret_access_key=secret.decode(),
    path_style=params.get_bool('ZooS3PathStyle'),
  )
  prefix = (params.get('ZooS3Prefix') or b'zoo').decode()
  scope = (params.get('ZooS3Scope') or b'minimal').decode()
  # runtime reload tolerates an expired key: warn, never stop uploading
  recipients = pgp.load_recipients(public_key.decode(), strict_expiry=False)
  return s3, prefix, scope, recipients


def main(exit_event: threading.Event | None = None) -> None:
  if exit_event is None:
    exit_event = threading.Event()

  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception('failed to set core affinity')

  clear_locks(Paths.log_root())

  params = Params()
  config = zoo_config_from_params(params)
  if config is None:
    cloudlog.info('zoo uploader: missing configuration, not starting')
    return
  s3, prefix, scope, recipients = config
  uploader = ZooUploader(Paths.log_root(), s3, prefix, scope, recipients, params)
  # one ring sweep per boot; an offline failure retries on the upload trigger
  uploader.maybe_prune(force=True)

  sm = messaging.SubMaster(['deviceState'])
  backoff = 0.1
  while not exit_event.is_set():
    sm.update(0)
    offroad = params.get_bool('IsOffroad')
    network_type = sm['deviceState'].networkType
    if network_type == NetworkType.none:
      time.sleep(60 if offroad else 5)
      continue

    success = uploader.step(network_type.raw, sm['deviceState'].networkMetered)
    if success is None:
      backoff = 60 if offroad else 5
    elif success:
      backoff = 0.1
    else:
      backoff = min(backoff * 2, 120)
    time.sleep(backoff + random.uniform(0, backoff))


if __name__ == '__main__':
  main()
