"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fetching a large model's ONNX by its git-lfs oid.

The big model is a git-lfs object .lfsconfig excludes from the install: it is
0.8 to 1.8 GB and only a device with a Jetson attached needs it, so it is
fetched on demand. Which server has it varies (sunnypilot mirrors comma's
master objects, comma's own servers hold the rest), so each endpoint is asked
in turn.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import urllib.request
from collections.abc import Callable
from pathlib import Path

from jetlink.registry.lfs import LFS_ENDPOINTS as COMMA_ENDPOINTS  # the Jetson asks the same servers

from openpilot.common.swaglog import cloudlog

LFS_MEDIA_TYPE = 'application/vnd.git-lfs+json'
CONNECT_TIMEOUT = 30.0
CHUNK = 4 << 20


class LfsError(Exception):
  pass


def lfsconfig_endpoint(repo_root: Path) -> str | None:
  """The LFS url this checkout is configured for, if any."""
  try:
    text = (repo_root / '.lfsconfig').read_text()
  except OSError:
    return None
  for line in text.splitlines():
    key, sep, value = line.strip().partition('=')
    if sep and key.strip() == 'url':
      return value.strip() or None
  return None


def endpoints(repo_root: Path) -> list[str]:
  """Where to look, nearest first: the checkout's own server, then comma's."""
  out = []
  configured = lfsconfig_endpoint(repo_root)
  if configured:
    out.append(configured.removesuffix('/'))
  out.extend(e for e in COMMA_ENDPOINTS if e not in out)
  return out


def resolve(endpoint: str, oid: str, size: int, timeout: float = CONNECT_TIMEOUT) -> str | None:
  """Ask one LFS server for a download href, or None if it does not have it."""
  body = json.dumps({
    'operation': 'download',
    'transfers': ['basic'],
    'objects': [{'oid': oid, 'size': size}],
  }).encode()
  request = urllib.request.Request(f'{endpoint}/objects/batch', data=body, method='POST',
                                   headers={'Accept': LFS_MEDIA_TYPE, 'Content-Type': LFS_MEDIA_TYPE})
  try:
    with urllib.request.urlopen(request, timeout=timeout) as response:
      payload = json.load(response)
  except Exception:
    cloudlog.warning("jetlink: lfs batch failed at %s", endpoint, exc_info=True)
    return None

  for obj in payload.get('objects', []):
    if obj.get('oid') != oid:
      continue
    if 'error' in obj:
      cloudlog.warning("jetlink: %s has no %s (%s)", endpoint, oid[:16], obj['error'].get('message'))
      return None
    href = (obj.get('actions', {}).get('download') or {}).get('href')
    if href:
      return href
  return None


def download(href: str, oid: str, size: int, dest: Path,
             progress: Callable[[float], None] | None = None,
             should_stop: Callable[[], bool] | None = None) -> Path:
  """Stream to a .part file, hashing as we go, and only then take the name.

  A half-written model must never be where the next boot would hand it to TensorRT.
  """
  dest.parent.mkdir(parents=True, exist_ok=True)
  free = shutil.disk_usage(dest.parent).free
  if free < size + (64 << 20):
    raise LfsError(f"need {size >> 20} MB for the large model, {free >> 20} MB free")

  part = dest.with_name(dest.name + '.part')
  digest = hashlib.sha256()
  written = 0
  # whole percent only: the callback writes a param, and a gigabyte at 4 MB a
  # chunk would write it a few hundred times
  reported = -1
  try:
    with urllib.request.urlopen(href, timeout=CONNECT_TIMEOUT) as response, open(part, 'wb') as out:
      while True:
        if should_stop is not None and should_stop():
          raise LfsError("download interrupted")
        chunk = response.read(CHUNK)
        if not chunk:
          break
        out.write(chunk)
        digest.update(chunk)
        written += len(chunk)
        if progress is not None and size:
          percent = int(100 * written / size)
          if percent != reported:
            reported = percent
            progress(min(1.0, written / size))
  except LfsError:
    part.unlink(missing_ok=True)
    raise
  except Exception as e:
    part.unlink(missing_ok=True)
    raise LfsError(f"could not download the large model: {e}") from e

  if written != size:
    part.unlink(missing_ok=True)
    raise LfsError(f"large model is {written} bytes, expected {size}")
  if digest.hexdigest() != oid:
    part.unlink(missing_ok=True)
    raise LfsError(f"large model hashes to {digest.hexdigest()[:16]}, expected {oid[:16]}")

  part.replace(dest)
  return dest



def fetch_oid(oid: str, size: int, dest: Path, repo_root: Path,
              progress: Callable[[float], None] | None = None,
              should_stop: Callable[[], bool] | None = None) -> Path:
  """Materialise one lfs object by oid, from whichever server has it."""
  if dest.is_file() and dest.stat().st_size == size:
    return dest

  for endpoint in endpoints(repo_root):
    href = resolve(endpoint, oid, size)
    if href is None:
      continue
    cloudlog.warning("jetlink: fetching the large model (%d MB) from %s", size >> 20, endpoint)
    return download(href, oid, size, dest, progress=progress, should_stop=should_stop)

  raise LfsError(f"no configured LFS server has {oid[:16]}")
