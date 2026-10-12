#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set up the zoo private S3 archive on a laptop.

Prompts for the bucket and credentials, optionally validates a PGP public
key the same way the device will (empty = uploads stay unencrypted), PUTs
a probe object as a self-test, and prints the exact params to apply on
the device over SSH. The device credential should be put-only, scoped to
the archive prefix; an archive ring budget additionally needs list and
delete capabilities.

Usage (from the repo root, with requests installed):
  python tools/zoo/configure.py [--self-test]
"""

import argparse
import os
import secrets
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from openpilot.common.zoo import pgp
from openpilot.common.zoo.s3sig import S3Config, signed_put, parse_budget


def ask(prompt: str, default: str = '') -> str:
  suffix = f' [{default}]' if default else ''
  value = input(f'{prompt}{suffix}: ').strip()
  return value or default


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument('--self-test', action='store_true', help='PUT an encrypted probe object after validating')
  args = parser.parse_args()

  print('zoo private S3 archive setup')
  print('The device credential must be put-only, scoped to the bucket or prefix.')
  print('A ring budget additionally needs list and delete on the same key.')
  print()
  endpoint = ask('S3 endpoint (e.g. https://s3.us-east-1.amazonaws.com, or a MinIO URL)')
  region = ask('Region', 'us-east-1')
  bucket = ask('Bucket')
  path_style = ask('Path-style addressing? 1 for MinIO/R2, 0 for AWS virtual-host', '1') == '1'
  access_key_id = ask('Access key id')
  secret = ask('Secret access key')
  pubkey_path = os.path.expanduser(ask('Path to armored PGP public key, empty for no encryption (gpg --export -a KEYID > pubkey.asc)'))
  prefix = ask('Archive prefix inside the bucket', secrets.token_hex(8))
  scope = ask('Upload scope: minimal (qlog+qcam+crash+boot), full, or selected (kept routes only)', 'minimal')
  budget = ask('Archive ring budget, e.g. 8G: the device deletes oldest files to stay under it (empty = off)', '')
  if budget and parse_budget(budget) is None:
    print(f'archive ring budget not understood: {budget!r} (examples: 8G, 500M)')
    return 1

  if not all((endpoint, bucket, access_key_id, secret)):
    print('endpoint, bucket, and credentials are required')
    return 1
  if endpoint.startswith('http://'):
    print('WARNING: plain http means the signed request is a short-lived bearer credential')
    print('on that wire. Fine for a trusted-LAN MinIO; use https in production.')

  public_key = None
  recipients = None
  if pubkey_path:
    try:
      with open(pubkey_path) as f:
        public_key = f.read()
      recipients = pgp.load_recipients(public_key, strict_expiry=True)
      algos = ['cv25519' if r['key'].algo == pgp.ECDH_ALGO else f'RSA{r["key"].fields["rsa"].key_size}' for r in recipients]
      print(f'PGP key ok: {len(recipients)} usable encryption key(s) ({", ".join(algos)})')

      # a public key cannot prove the private half exists: write one probe the
      # user decrypts, so a wrong-key setup or an untested backup key fails
      # here instead of months later
      key_ids = ' '.join(r['key'].key_id().hex() for r in recipients)
      probe_path = os.path.join(os.path.dirname(os.path.abspath(pubkey_path)), 'zoo-key-check.asc')
      with open(probe_path, 'w') as f:
        f.write(pgp.armor(pgp.encrypt(f'zoo key check ok: {key_ids}'.encode(), recipients)))
      print(f'Key check: gpg -d {probe_path} must print "zoo key check ok" and your key id')
    except (OSError, pgp.PgpError) as e:
      print(f'PGP key rejected: {e}')
      return 1
  else:
    print('No PGP key: uploads store unencrypted, readable to anyone with')
    print('access to the bucket (the provider included).')

  cfg = S3Config(endpoint=endpoint, region=region, bucket=bucket, access_key_id=access_key_id, secret_access_key=secret, path_style=path_style)
  if args.self_test:
    import requests

    # probe objects sort by wall clock
    probe_key = f'{prefix}/_selftest/{int(time.time())}'  # noqa: TID251
    probe = pgp.encrypt(b'zoo self-test', recipients) if recipients else b'zoo self-test'
    url, headers, body = signed_put(cfg, probe_key, probe)
    r = requests.put(url, headers=headers, data=body, timeout=15)
    if r.status_code == 200:
      print(f'self-test ok: probe stored at {probe_key}')
      print('(put-only credentials cannot delete it; remove it with your admin credentials)')
    else:
      print(f'self-test FAILED: HTTP {r.status_code}: {r.text[:200]}')
      return 1

  print()
  print('Apply on the device over SSH (restart after the last line):')
  values = [
    ('ZooS3Endpoint', endpoint),
    ('ZooS3Region', region),
    ('ZooS3Bucket', bucket),
    ('ZooS3PathStyle', '1' if path_style else '0'),
    ('ZooS3AccessKeyId', access_key_id),
    ('ZooS3SecretAccessKey', secret),
  ]
  if public_key:
    values.append(('ZooS3Encryption', '1'))
    values.append(('ZooS3PublicKey', public_key))
  values += [
    ('ZooS3Prefix', prefix),
    ('ZooS3Scope', scope),
    ('ZooS3Enabled', '1'),
  ]
  if budget:
    values.append(('ZooS3ArchiveBudget', budget))
  for name, value in values:
    if name == 'ZooS3PublicKey':
      print(f"  cat > /data/params/d/{name} <<'EOF'\n{value}EOF")
    else:
      print(f"  printf '%s' '{value}' > /data/params/d/{name}")
  print()
  if budget:
    print('Ring on: the device key needs listFiles and deleteFiles too, and bucket')
    print('versioning with an expiry window protects the archive from a leaked key.')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
