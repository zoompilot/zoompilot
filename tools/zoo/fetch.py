#!/usr/bin/env python3
"""Read the zoo private S3 archive from a laptop.

With no frontend anywhere, this is the friendly read path: list objects,
download, gpg-decrypt, and decompress a route's files back to their
original names. Uses read/list credentials (the device's put-only
credential cannot read anything back). keep and unkeep write the route
selection markers, so they need a write-capable credential.

Usage (from the repo root, with requests installed; zstandard optional):
  python tools/zoo/fetch.py list [--prefix P] [options]
  python tools/zoo/fetch.py get OBJECT_KEY [-o OUT] [options]
  python tools/zoo/fetch.py route DIR [options]
  python tools/zoo/fetch.py kept [options]
  python tools/zoo/fetch.py keep ROUTE [options]
  python tools/zoo/fetch.py unkeep ROUTE [options]

Options: --endpoint --region --bucket --access-key-id --secret-access-key
--path-style --prefix (archive prefix), or the ZOO_S3_* environment vars.
"""

import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from openpilot.common.zoo.s3sig import S3Config, signed_put, signed_request

NS = '{http://s3.amazonaws.com/doc/2006-03-01/}'


def gpg_decrypt(data: bytes) -> bytes:
  proc = subprocess.run(['gpg', '--batch', '-d'], input=data, capture_output=True)
  if proc.returncode != 0:
    raise RuntimeError(f'gpg decrypt failed: {proc.stderr.decode()[-200:]}')
  return proc.stdout


def maybe_decompress(data: bytes, name: str) -> tuple[bytes, str]:
  if name.endswith('.zst'):
    try:
      import zstandard
    except ImportError as e:
      raise RuntimeError('zstandard is required to decompress .zst files (pip install zstandard)') from e
    return zstandard.ZstdDecompressor().decompress(data), name[:-4]
  return data, name


def list_objects(cfg: S3Config, archive_prefix: str, cli_prefix: str) -> list[str]:
  params = {'list-type': '2', 'prefix': f'{archive_prefix}/{cli_prefix}'.rstrip('/'), 'max-keys': '1000'}
  objects = []
  while True:
    import requests

    url, headers = signed_request(cfg, 'GET', '', query=params)
    r = requests.get(url, headers=headers, timeout=30)
    r.raise_for_status()
    root = ET.fromstring(r.content)
    for contents in root.findall(f'{NS}Contents'):
      objects.append(contents.findtext(f'{NS}Key'))
    token = root.findtext(f'{NS}NextContinuationToken')
    if token is None:
      return objects
    params['continuation-token'] = token


def get_object(cfg: S3Config, key: str, out_path: str) -> str:
  import requests

  url, headers = signed_request(cfg, 'GET', key)
  r = requests.get(url, headers=headers, timeout=60)
  r.raise_for_status()

  name = key.rsplit('/', 1)[-1]
  if not name.endswith('.gpg'):
    raise RuntimeError(f'not a zoo archive object: {key}')
  data, name = maybe_decompress(gpg_decrypt(r.content), name[:-4])
  with open(os.path.expanduser(out_path or name), 'wb') as f:
    f.write(data)
  return out_path or name


def kept_routes(cfg: S3Config, archive_prefix: str) -> list[str]:
  marker_prefix = f'{archive_prefix}/keep/' if archive_prefix else 'keep/'
  names = set()
  for key in list_objects(cfg, archive_prefix, 'keep'):
    name = key.removeprefix(marker_prefix).split('/', 1)[0]
    if name:
      names.add(name)
  return sorted(names)


def set_kept(cfg: S3Config, key: str, keep: bool) -> None:
  """Write or delete a keep marker object; needs write credentials."""
  import requests

  if keep:
    url, headers, body = signed_put(cfg, key, b'')
    r = requests.put(url, headers=headers, data=body, timeout=30)
  else:
    url, headers = signed_request(cfg, 'DELETE', key)
    r = requests.delete(url, headers=headers, timeout=30)
  r.raise_for_status()


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument('command', choices=['list', 'get', 'route', 'kept', 'keep', 'unkeep'])
  parser.add_argument('target', nargs='?', help='object key (get), directory name (route), or route name (keep/unkeep)')
  parser.add_argument('--prefix', default='', help='key filter for list')
  parser.add_argument('-o', '--out')
  parser.add_argument('--endpoint', default=os.environ.get('ZOO_S3_ENDPOINT', ''))
  parser.add_argument('--region', default=os.environ.get('ZOO_S3_REGION', 'us-east-1'))
  parser.add_argument('--bucket', default=os.environ.get('ZOO_S3_BUCKET', ''))
  parser.add_argument('--access-key-id', default=os.environ.get('ZOO_S3_ACCESS_KEY_ID', ''))
  parser.add_argument('--secret-access-key', default=os.environ.get('ZOO_S3_SECRET_ACCESS_KEY', ''))
  parser.add_argument('--path-style', default=os.environ.get('ZOO_S3_PATH_STYLE', '1'))
  args = parser.parse_args()

  if not all((args.endpoint, args.bucket, args.access_key_id, args.secret_access_key)):
    parser.error('endpoint, bucket, and credentials are required (flags or ZOO_S3_* env)')

  cfg = S3Config(
    endpoint=args.endpoint,
    region=args.region,
    bucket=args.bucket,
    access_key_id=args.access_key_id,
    secret_access_key=args.secret_access_key,
    path_style=args.path_style == '1',
  )
  # everything under the archive prefix; strip "prefix/" from display
  archive_prefix = os.environ.get('ZOO_S3_ARCHIVE_PREFIX', '')

  if args.command == 'list':
    for key in list_objects(cfg, archive_prefix, args.prefix):
      print(key.removeprefix(f'{archive_prefix}/'))
    return 0

  if args.command == 'kept':
    for name in kept_routes(cfg, archive_prefix):
      print(name)
    return 0

  if args.command in ('keep', 'unkeep'):
    route = (args.target or '').strip().strip('/')
    if not route or '/' in route:
      parser.error('give one route name, for example 000001a3--c20ba54385')
    marker_key = f'{archive_prefix}/keep/{route}'.lstrip('/')
    set_kept(cfg, marker_key, args.command == 'keep')
    print(f"{'kept' if args.command == 'keep' else 'unkept'}: {route}")
    return 0

  if not args.target:
    parser.error(f'{args.command} needs a target')
  key = f'{archive_prefix}/{args.target}'.strip('/') if archive_prefix else args.target

  if args.command == 'get':
    print(get_object(cfg, key, args.out))
    return 0

  for obj in list_objects(cfg, archive_prefix, args.target):
    name = obj.removeprefix(f'{archive_prefix}/')
    parent = os.path.dirname(name)
    if parent:
      os.makedirs(parent, exist_ok=True)
    print(get_object(cfg, obj, name))
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
