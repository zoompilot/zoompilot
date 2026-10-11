"""Header-signed AWS Signature Version 4 PUT for the zoo S3 archive.

Signs the payload hash and an x-amz-checksum-sha256 header into the
Authorization header, so a spooled ciphertext of known length and digest
uploads in one request with no presigned URL and no boto3. Works with any
S3-compatible endpoint: AWS S3, Cloudflare R2, Backblaze B2, MinIO.
"""

import base64
import datetime
import hashlib
import hmac
import re
from dataclasses import dataclass

ALGORITHM = 'AWS4-HMAC-SHA256'
UNRESERVED = re.compile(r'[A-Za-z0-9\-._~]')


@dataclass
class S3Config:
  endpoint: str  # e.g. https://s3.us-east-1.amazonaws.com or http://127.0.0.1:9000
  region: str
  bucket: str
  access_key_id: str
  secret_access_key: str
  path_style: bool = True


def _uri_encode(value: str, keep_slash: bool = True) -> str:
  # S3 rule: RFC 3986 unreserved characters pass, everything else is %XX
  # upper hex; a leading '/' per segment stays when keep_slash
  out = []
  for ch in value:
    if UNRESERVED.match(ch) or (ch == '/' and keep_slash):
      out.append(ch)
    else:
      out.extend(f'%{b:02X}' for b in ch.encode('utf-8'))
  return ''.join(out)


def _hmac(key: bytes, msg: str) -> bytes:
  return hmac.new(key, msg.encode(), hashlib.sha256).digest()


def _split_endpoint(endpoint: str, bucket: str, path_style: bool) -> tuple[str, str, bool]:
  """Return (scheme, host_with_optional_port, path_style_effective)."""
  if '://' not in endpoint:
    endpoint = 'https://' + endpoint
  scheme, netloc = endpoint.split('://', 1)
  netloc = netloc.rstrip('/')
  host = netloc.split(':', 1)[0]
  # an IP or custom-port endpoint cannot do virtual-host addressing
  effective_path_style = path_style or re.match(r'^\d+\.\d+\.\d+\.\d+$', host) is not None
  if not effective_path_style:
    netloc = f'{bucket}.{netloc}'
  return scheme, netloc, effective_path_style


def signed_put_file(cfg: S3Config, key: str, path: str, timestamp: datetime.datetime | None = None) -> tuple[str, dict]:
  """Sign a PUT of a spooled file; hash streams from disk.

  Returns (url, headers). Open the file and pass it as requests' data; the
  signed content hash and checksum were computed over the same bytes.
  """
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    while chunk := f.read(1 << 20):
      h.update(chunk)
  return signed_request(cfg, 'PUT', key, payload_hash=h, timestamp=timestamp)


def signed_put(cfg: S3Config, key: str, payload: bytes, timestamp: datetime.datetime | None = None) -> tuple[str, dict, bytes]:
  """Sign a PUT of payload to key.

  Returns (url, headers, body). The body is the same bytes passed in; the
  SHA-256 of it is signed and an x-amz-checksum-sha256 header rides along.
  Pass the spooled ciphertext here so the signature matches what goes on
  the wire byte for byte.
  """
  url, headers = signed_request(cfg, 'PUT', key, payload_hash=hashlib.sha256(payload), timestamp=timestamp)
  return url, headers, payload


def signed_request(cfg: S3Config, method: str, key: str = '', query: dict[str, str] | None = None,
                   payload_hash: 'hashlib._Hash | None' = None,
                   timestamp: datetime.datetime | None = None) -> tuple[str, dict]:
  """Sign any request: PUT (with checksum header), GET, LIST, or DELETE.

  Returns (url, headers). The payload hash defaults to the empty-string hash
  for GET, LIST, and DELETE. Canonical query is sorted and S3-encoded per
  SigV4.
  """
  # signing needs wall-clock time by protocol
  now = timestamp or datetime.datetime.now(datetime.UTC).replace(microsecond=0)
  amz_date = now.strftime('%Y%m%dT%H%M%SZ')
  date_stamp = now.strftime('%Y%m%d')

  scheme, host, path_style = _split_endpoint(cfg.endpoint, cfg.bucket, cfg.path_style)
  canonical_uri = f'/{cfg.bucket}/{key}' if path_style else f'/{key}'
  canonical_uri = _uri_encode(canonical_uri)

  query = query or {}
  canonical_query = '&'.join(
    f'{_uri_encode(k, keep_slash=False)}={_uri_encode(v, keep_slash=False)}'
    for k, v in sorted(query.items()))
  url = f'{scheme}://{host}{canonical_uri}'
  if canonical_query:
    url += f'?{canonical_query}'

  hash_obj = payload_hash or hashlib.sha256()
  payload_hash_hex = hash_obj.hexdigest()
  headers = {
    'Host': host,
    'x-amz-date': amz_date,
    'x-amz-content-sha256': payload_hash_hex,
  }
  if method == 'PUT':
    headers['x-amz-checksum-sha256'] = base64.b64encode(hash_obj.digest()).decode()
  signed = ';'.join(sorted(k.lower() for k in headers))
  canonical_headers = ''.join(f'{k.lower()}:{headers[k].strip()}\n' for k in sorted(headers, key=str.lower))

  payload_hash = payload_hash_hex

  canonical_request = '\n'.join(
    [
      method,
      canonical_uri,
      canonical_query,
      canonical_headers,
      signed,
      payload_hash,
    ]
  )
  scope = f'{date_stamp}/{cfg.region}/s3/aws4_request'
  string_to_sign = '\n'.join(
    [
      ALGORITHM,
      amz_date,
      scope,
      hashlib.sha256(canonical_request.encode()).hexdigest(),
    ]
  )
  k_date = hmac.new(f'AWS4{cfg.secret_access_key}'.encode(), date_stamp.encode(), hashlib.sha256).digest()
  k_region = hmac.new(k_date, cfg.region.encode(), hashlib.sha256).digest()
  k_service = hmac.new(k_region, b's3', hashlib.sha256).digest()
  k_signing = hmac.new(k_service, b'aws4_request', hashlib.sha256).digest()
  signature = hmac.new(k_signing, string_to_sign.encode(), hashlib.sha256).hexdigest()

  headers['Authorization'] = f'{ALGORITHM} Credential={cfg.access_key_id}/{scope}, SignedHeaders={signed}, Signature={signature}'
  return url, headers


def parse_budget(text: str) -> int | None:
  """Parse an archive ring budget like '8G', '500M', '2K', or plain bytes.

  Returns None for an empty or unparseable value; the ring stays off.
  """
  text = (text or '').strip()
  if not text:
    return None
  m = re.fullmatch(r'(\d+(?:\.\d+)?)\s*([KMG]?)(?:B)?', text, re.IGNORECASE)
  if m is None:
    return None
  value = float(m.group(1))
  value *= {'': 1, 'K': 1 << 10, 'M': 1 << 20, 'G': 1 << 30}[m.group(2).upper()]
  return int(value)
