"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared helpers for the zoo private S3 archive.
"""

import json
import os
import re

UPLOAD_ROUTES_PARAM = 'ZooS3UploadRoutes'

_COUNTER_SPLIT = re.compile(r'-+')


def list_route_dirs(root: str) -> list[str]:
  """Route directory names in the log root, oldest first."""
  dirs = []
  try:
    entries = os.listdir(root)
  except OSError:
    return dirs
  for name in entries:
    if not os.path.isdir(os.path.join(root, name)):
      continue
    try:
      int(name.split('--', 1)[0], 16)
    except ValueError:
      continue  # boot/, crash/, spool, and anything else that is not a route
    dirs.append(name)
  return sorted(dirs)


def latest_route_dir(root: str) -> str | None:
  dirs = list_route_dirs(root)
  return dirs[-1] if dirs else None


def normalize_route_request(text: str, route_dirs: list[str]) -> str | None:
  """Match typed text to one route directory name, or None.

  Accepts the drive number alone, with or without leading zeros, in any
  case, or the full route id. A pasted id that carries a dongle prefix or
  a different separator still works: only the last segment counts.
  """
  token = (text or '').strip()
  if not token:
    return None
  token = re.split(r'[/|\s]', token)[-1].lower()
  parts = _COUNTER_SPLIT.split(token, maxsplit=1)
  counter_text = parts[0]
  suffix = parts[1] if len(parts) > 1 else None
  try:
    counter = int(counter_text, 16)
  except ValueError:
    return None

  for route in route_dirs:
    route_counter, _, route_suffix = route.lower().partition('--')
    try:
      if int(route_counter, 16) != counter:
        continue
    except ValueError:
      continue
    if suffix is None or route_suffix == suffix:
      return route
  return None


def load_upload_requests(params) -> list[dict]:
  """Pending upload requests as [{'route': dir name, 'uploaded_any': bool}]."""
  try:
    raw = json.loads(params.get(UPLOAD_ROUTES_PARAM) or b'[]')
  except (ValueError, TypeError):
    return []
  if not isinstance(raw, list):
    return []
  return [r for r in raw if isinstance(r, dict) and isinstance(r.get('route'), str) and r['route']]


def save_upload_requests(params, requests: list[dict]) -> None:
  params.put(UPLOAD_ROUTES_PARAM, json.dumps(requests))


def add_upload_request(params, route: str) -> bool:
  """Queue one route; False when it is already queued."""
  requests = load_upload_requests(params)
  if any(r['route'] == route for r in requests):
    return False
  requests.append({'route': route, 'uploaded_any': False})
  save_upload_requests(params, requests)
  return True
