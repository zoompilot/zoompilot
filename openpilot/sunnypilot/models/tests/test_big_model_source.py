"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.models import big_model_source as B

POINTER = b"version https://git-lfs.github.com/spec/v1\noid sha256:" + b"a" * 64 + b"\nsize 766040736\n"
MODEL = b'\x80\x05pkl'


def model(tmp_path: Path, body: bytes) -> Path:
  path = tmp_path / 'openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl'
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_bytes(body)
  return path


def test_a_pointer_is_told_from_the_model(tmp_path):
  assert B.is_lfs_pointer(model(tmp_path, POINTER))
  assert not B.is_lfs_pointer(model(tmp_path, MODEL))
  assert not B.is_lfs_pointer(tmp_path / 'missing.pkl')


def test_a_pointer_is_not_compiled(tmp_path):
  # the big warps are tracked, so only the model tells a fetched install from a pointer
  from openpilot.selfdrive.modeld import helpers
  path = model(tmp_path, POINTER)
  for size in ('1344x760', '1928x1208'):
    (path.parent / f'big_driving_warp_{size}_tinygrad.pkl').write_bytes(b'w')
  with mock.patch.object(helpers, 'MODELS_DIR', path.parent), mock.patch.object(helpers, 'modeld_pkl_path', lambda chestnut: path):
    assert not helpers.chestnut_compiled()
    path.write_bytes(MODEL)
    assert helpers.chestnut_compiled()
