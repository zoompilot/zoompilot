import base64
import datetime
import hashlib
import hmac
import http.server
import threading

import pytest
import requests

from openpilot.common.zoo.s3sig import S3Config, signed_put, signed_request, parse_budget, _uri_encode

AWS_KEY = 'zooAccessKeyId00000000'
AWS_SECRET = 'zooSecretAccessKey0000000000000000000000'


def cfg(endpoint, path_style=True):
  return S3Config(endpoint=endpoint, region='us-east-1', bucket='zoo-archive', access_key_id=AWS_KEY, secret_access_key=AWS_SECRET, path_style=path_style)


TS = datetime.datetime(2026, 10, 9, 12, 0, 0)


def test_uri_encode():
  assert _uri_encode('/a b/c+d') == '/a%20b/c%2Bd'
  assert _uri_encode('seg/ü') == 'seg/%C3%BC'
  assert _uri_encode('plain-._~name', keep_slash=False) == 'plain-._~name'
  assert _uri_encode('a/b', keep_slash=False) == 'a%2Fb'


def test_parse_budget():
  assert parse_budget('8G') == 8 << 30
  assert parse_budget('8GB') == 8 << 30
  assert parse_budget('500M') == 500 << 20
  assert parse_budget('2k') == 2 << 10
  assert parse_budget('1073741824') == 1073741824
  assert parse_budget('1.5G') == int(1.5 * (1 << 30))
  assert parse_budget(' 8G ') == 8 << 30
  assert parse_budget('') is None
  assert parse_budget(None) is None
  assert parse_budget('lots') is None
  assert parse_budget('-5G') is None
  assert parse_budget('8T') is None


def test_path_style_and_virtual_host_urls():
  url, headers, _ = signed_put(cfg('http://127.0.0.1:9000'), 'k.bin', b'x', TS)
  assert url == 'http://127.0.0.1:9000/zoo-archive/k.bin'
  url, _, _ = signed_put(cfg('https://s3.us-east-1.amazonaws.com', path_style=False), 'k.bin', b'x', TS)
  assert url == 'https://zoo-archive.s3.us-east-1.amazonaws.com/k.bin'
  # an IP endpoint never gets virtual-host style
  url, _, _ = signed_put(cfg('http://127.0.0.1:9000', path_style=False), 'k.bin', b'x', TS)
  assert url == 'http://127.0.0.1:9000/zoo-archive/k.bin'


def test_signature_is_deterministic_and_covers_payload():
  _, h1, _ = signed_put(cfg('https://s3.amazonaws.com'), 'k', b'one', TS)
  _, h2, _ = signed_put(cfg('https://s3.amazonaws.com'), 'k', b'one', TS)
  _, h3, _ = signed_put(cfg('https://s3.amazonaws.com'), 'k', b'two', TS)
  assert h1['Authorization'] == h2['Authorization']
  assert h1['Authorization'] != h3['Authorization']
  assert h1['x-amz-content-sha256'] == hashlib.sha256(b'one').hexdigest()
  assert h1['x-amz-checksum-sha256'] == base64.b64encode(hashlib.sha256(b'one').digest()).decode()


def test_header_shape():
  _, headers, _ = signed_put(cfg('http://127.0.0.1:9000'), 'k', b'x', TS)
  auth = headers['Authorization']
  assert auth.startswith('AWS4-HMAC-SHA256 Credential=zooAccessKeyId00000000/20261009/us-east-1/s3/aws4_request')
  assert 'SignedHeaders=host;x-amz-checksum-sha256;x-amz-content-sha256;x-amz-date' in auth


def test_sigv4_matches_botocore():
  # cross-check against AWS's own signer: same request, same timestamp,
  # identical Authorization headers. Skips unless botocore is importable
  # (dev machines install it in the oracle venv).
  pytest.importorskip('botocore')
  from botocore.awsrequest import AWSRequest
  from botocore.credentials import Credentials
  from botocore.auth import S3SigV4Auth

  payload = b'ciphertext-bytes-for-reference-comparison'
  c = cfg('https://s3.us-east-1.amazonaws.com')
  url, _, _ = signed_put(c, 'zoo-arch/dir/file.bin.gpg', payload, TS)

  req = AWSRequest(method='PUT', url=url, data=payload)
  req.headers['x-amz-checksum-sha256'] = base64.b64encode(hashlib.sha256(payload).digest()).decode()
  S3SigV4Auth(Credentials(AWS_KEY, AWS_SECRET), 's3', 'us-east-1').add_auth(req)

  _, mine, _ = signed_put(
    c, 'zoo-arch/dir/file.bin.gpg', payload, datetime.datetime.strptime(req.headers['X-Amz-Date'], '%Y%m%dT%H%M%SZ').replace(tzinfo=datetime.UTC)
  )
  assert mine['Authorization'] == req.headers['Authorization']


class _VerifyingStub(http.server.BaseHTTPRequestHandler):
  # independent SigV4 verifier: rebuild the canonical request from what
  # actually arrives on the wire and recompute the signature
  failure = None

  def _expected_signature(self, method: str, signed_headers: str, canonical_headers: str, payload_hash_hex: str) -> str:
    auth = self.headers.get('Authorization', '')
    amz_date = self.headers.get('x-amz-date', '')
    creq = '\n'.join([method, self.path, '', canonical_headers, signed_headers, payload_hash_hex])
    date_stamp, region, _ = auth.split('Credential=')[1].split('/')[1:4]
    scope_str = f'{date_stamp}/{region}/s3/aws4_request'
    sts = '\n'.join(['AWS4-HMAC-SHA256', amz_date, scope_str, hashlib.sha256(creq.encode()).hexdigest()])
    k = hmac.new(f'AWS4{AWS_SECRET}'.encode(), date_stamp.encode(), hashlib.sha256).digest()
    k = hmac.new(k, region.encode(), hashlib.sha256).digest()
    k = hmac.new(k, b's3', hashlib.sha256).digest()
    k = hmac.new(k, b'aws4_request', hashlib.sha256).digest()
    return hmac.new(k, sts.encode(), hashlib.sha256).hexdigest()

  def do_PUT(self):
    length = int(self.headers.get('Content-Length', 0))
    body = self.rfile.read(length)
    auth = self.headers.get('Authorization', '')
    payload_hash = hashlib.sha256(body).hexdigest()
    signed = 'host;x-amz-checksum-sha256;x-amz-content-sha256;x-amz-date'
    canonical_headers = ''.join(
      [
        f'host:{self.headers["Host"]}\n',
        f'x-amz-checksum-sha256:{self.headers["x-amz-checksum-sha256"]}\n',
        f'x-amz-content-sha256:{self.headers["x-amz-content-sha256"]}\n',
        f'x-amz-date:{self.headers["x-amz-date"]}\n',
      ]
    )
    expected = self._expected_signature('PUT', signed, canonical_headers, payload_hash)

    checksum_ok = self.headers['x-amz-checksum-sha256'] == base64.b64encode(hashlib.sha256(body).digest()).decode()
    sig_ok = f'Signature={expected}' in auth
    if sig_ok and checksum_ok:
      self.send_response(200)
    elif not checksum_ok:
      self.send_response(400)
    else:
      self.send_response(403)
    self.end_headers()

  def do_DELETE(self):
    # no body: the canonical payload hash is the empty-string hash
    payload_hash = hashlib.sha256(b'').hexdigest()
    signed = 'host;x-amz-content-sha256;x-amz-date'
    canonical_headers = ''.join(
      [
        f'host:{self.headers["Host"]}\n',
        f'x-amz-content-sha256:{self.headers["x-amz-content-sha256"]}\n',
        f'x-amz-date:{self.headers["x-amz-date"]}\n',
      ]
    )
    expected = self._expected_signature('DELETE', signed, canonical_headers, payload_hash)
    self.send_response(204 if f'Signature={expected}' in self.headers.get('Authorization', '') else 403)
    self.end_headers()

  def log_message(self, *args):
    pass


@pytest.fixture(scope='module')
def stub():
  server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), _VerifyingStub)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  yield f'http://127.0.0.1:{server.server_address[1]}'
  server.shutdown()


def test_put_signature_verifies_on_the_wire(stub):
  c = cfg(stub)
  url, headers, body = signed_put(c, 'zoo-arch/dir/file~name +odd.bin.gpg', b'ciphertext-bytes', TS)
  r = requests.put(url, headers=headers, data=body)
  assert r.status_code == 200, r.text

  # tampered signature rejected
  bad = dict(headers)
  bad['Authorization'] = bad['Authorization'][:-4] + 'beef'
  r = requests.put(url, headers=bad, data=body)
  assert r.status_code == 403

  # checksum not matching the body rejected
  badck = dict(headers)
  badck['x-amz-checksum-sha256'] = 'AAAA' + badck['x-amz-checksum-sha256'][4:]
  r = requests.put(url, headers=badck, data=b'other-bytes')
  assert r.status_code == 400


def test_delete_signature_verifies_on_the_wire(stub):
  c = cfg(stub)
  url, headers = signed_request(c, 'DELETE', 'zoo-arch/old/route/qlog.gpg', timestamp=TS)
  r = requests.delete(url, headers=headers)
  assert r.status_code == 204

  # tampered signature rejected
  bad = dict(headers)
  bad['Authorization'] = bad['Authorization'][:-4] + 'beef'
  r = requests.delete(url, headers=bad)
  assert r.status_code == 403
