"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.accelerators.jetlink import lfs

BODY = b'onnx' * 4096
OID = hashlib.sha256(BODY).hexdigest()
SIZE = len(BODY)

POINTER = f"""version https://git-lfs.github.com/spec/v1
oid sha256:{OID}
size {SIZE}
"""


class FakeResponse:
  def __init__(self, payload: bytes):
    self._payload = payload
    self._pos = 0

  def read(self, n=None):
    if n is None:
      n, self._pos = len(self._payload) - self._pos, len(self._payload)
      return self._payload[-n:] if n else b''
    chunk = self._payload[self._pos:self._pos + n]
    self._pos += len(chunk)
    return chunk

  def __enter__(self):
    return self

  def __exit__(self, *_):
    return False


class TestEndpoints(unittest.TestCase):
  def setUp(self):
    self.root = Path(tempfile.mkdtemp())

  def test_configured_endpoint_comes_first(self):
    (self.root / '.lfsconfig').write_text('[lfs]\n\turl = https://example.com/info/lfs\n')
    assert lfs.endpoints(self.root) == ['https://example.com/info/lfs', *lfs.COMMA_ENDPOINTS]

  def test_commas_endpoints_are_always_there(self):
    # sunnypilot's mirror carries master's objects; only comma's servers have
    # the PR-branch ones, and only GitLab has the older ones
    assert lfs.endpoints(self.root) == list(lfs.COMMA_ENDPOINTS)

  def test_no_duplicate_when_already_commas(self):
    (self.root / '.lfsconfig').write_text(f'[lfs]\n\turl = {lfs.COMMA_ENDPOINTS[0]}\n')
    assert lfs.endpoints(self.root) == list(lfs.COMMA_ENDPOINTS)


class TestResolve(unittest.TestCase):
  def urlopen_returning(self, payload: dict):
    return mock.patch.object(lfs.urllib.request, 'urlopen',
                             return_value=FakeResponse(json.dumps(payload).encode()))

  def test_returns_the_href(self):
    with self.urlopen_returning({'objects': [{'oid': OID, 'actions': {'download': {'href': 'https://x/y'}}}]}):
      assert lfs.resolve('https://e/info/lfs', OID, SIZE) == 'https://x/y'

  def test_an_error_object_is_a_miss_not_a_raise(self):
    with self.urlopen_returning({'objects': [{'oid': OID, 'error': {'code': 404, 'message': 'nope'}}]}):
      assert lfs.resolve('https://e/info/lfs', OID, SIZE) is None

  def test_a_different_oid_is_a_miss(self):
    with self.urlopen_returning({'objects': [{'oid': 'deadbeef', 'actions': {'download': {'href': 'https://x/y'}}}]}):
      assert lfs.resolve('https://e/info/lfs', OID, SIZE) is None

  def test_a_dead_server_is_a_miss_not_a_raise(self):
    # One unreachable endpoint must not stop us asking the next.
    with mock.patch.object(lfs.urllib.request, 'urlopen', side_effect=OSError('refused')):
      assert lfs.resolve('https://e/info/lfs', OID, SIZE) is None


class TestDownload(unittest.TestCase):
  def setUp(self):
    self.tmp = Path(tempfile.mkdtemp())
    self.dest = self.tmp / 'big.onnx'

  def urlopen_returning(self, payload: bytes):
    return mock.patch.object(lfs.urllib.request, 'urlopen', return_value=FakeResponse(payload))

  def test_writes_and_verifies(self):
    with self.urlopen_returning(BODY):
      assert lfs.download('https://x/y', OID, SIZE, self.dest) == self.dest
    assert self.dest.read_bytes() == BODY

  def test_a_corrupt_body_leaves_nothing_behind(self):
    # half a model that TensorRT would try to parse is the one outcome worth being paranoid about
    corrupt = b'x' * SIZE
    with self.urlopen_returning(corrupt), self.assertRaises(lfs.LfsError):
      lfs.download('https://x/y', OID, SIZE, self.dest)
    assert not self.dest.exists()
    assert not list(self.tmp.glob('*.part'))

  def test_a_short_body_leaves_nothing_behind(self):
    with self.urlopen_returning(BODY[:100]), self.assertRaises(lfs.LfsError):
      lfs.download('https://x/y', OID, SIZE, self.dest)
    assert not self.dest.exists()
    assert not list(self.tmp.glob('*.part'))

  def test_stopping_leaves_nothing_behind(self):
    with self.urlopen_returning(BODY), self.assertRaises(lfs.LfsError):
      lfs.download('https://x/y', OID, SIZE, self.dest, should_stop=lambda: True)
    assert not list(self.tmp.glob('*.part'))

  def test_refuses_without_room(self):
    with mock.patch.object(lfs.shutil, 'disk_usage') as usage:
      usage.return_value = mock.Mock(free=1024)
      with self.assertRaises(lfs.LfsError):
        lfs.download('https://x/y', OID, 1 << 30, self.dest)

  def test_progress_reaches_one(self):
    seen = []
    with self.urlopen_returning(BODY):
      lfs.download('https://x/y', OID, SIZE, self.dest, progress=seen.append)
    assert seen and seen[-1] == 1.0
    assert seen == sorted(seen)


class TestFetchOid(unittest.TestCase):
  def setUp(self):
    self.tmp = Path(tempfile.mkdtemp())
    self.dest = self.tmp / 'models' / 'out.onnx'   # a directory the downloader has to make

  def test_already_fetched_is_returned_as_is(self):
    self.dest.parent.mkdir()
    self.dest.write_bytes(BODY)
    with mock.patch.object(lfs, 'resolve') as resolve:
      assert lfs.fetch_oid(OID, SIZE, self.dest, self.tmp) == self.dest
      resolve.assert_not_called()

  def test_falls_through_to_the_next_endpoint(self):
    (self.tmp / '.lfsconfig').write_text('[lfs]\n\turl = https://dead.example/info/lfs\n')
    with mock.patch.object(lfs, 'resolve', side_effect=[None, 'https://x/y']) as resolve, \
         mock.patch.object(lfs.urllib.request, 'urlopen', return_value=FakeResponse(BODY)):
      assert lfs.fetch_oid(OID, SIZE, self.dest, self.tmp) == self.dest
    assert self.dest.read_bytes() == BODY
    assert [call.args[0] for call in resolve.call_args_list] == ['https://dead.example/info/lfs', lfs.COMMA_ENDPOINTS[0]]

  def test_nowhere_to_get_it_raises(self):
    with mock.patch.object(lfs, 'resolve', return_value=None), self.assertRaises(lfs.LfsError):
      lfs.fetch_oid(OID, SIZE, self.dest, self.tmp)


if __name__ == '__main__':
  unittest.main()
