"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import subprocess
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.models import big_model_source as B

POINTER = b"version https://git-lfs.github.com/spec/v1\noid sha256:" + b"a" * 64 + b"\nsize 766040736\n"


def model(tmp_path: Path, body: bytes) -> Path:
  path = tmp_path / 'openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx'
  path.parent.mkdir(parents=True)
  path.write_bytes(body)
  return path


def test_a_model_already_there_is_not_fetched(tmp_path):
  path = model(tmp_path, b'\x08\x07onnx')
  with mock.patch.object(B.subprocess, 'run') as run:
    assert B.ensure_big_onnx(path, str(tmp_path))
  run.assert_not_called()


def test_a_pointer_is_fetched_past_the_exclude(tmp_path):
  path = model(tmp_path, POINTER)
  with mock.patch.object(B.subprocess, 'run', side_effect=lambda *a, **k: path.write_bytes(b'\x08\x07onnx')) as run:
    assert B.ensure_big_onnx(path, str(tmp_path))
  args, kwargs = run.call_args
  assert args[0] == ['git', 'lfs', 'pull', '--include', 'openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx',
                     '--exclude', '']
  assert kwargs['cwd'] == str(tmp_path) and kwargs['check']


def test_a_failed_fetch_skips_the_big_model_instead_of_failing_the_build(tmp_path):
  path = model(tmp_path, POINTER)
  for failure in (subprocess.CalledProcessError(2, 'git'), subprocess.TimeoutExpired('git', 1), FileNotFoundError('git')):
    with mock.patch.object(B.subprocess, 'run', side_effect=failure):
      assert not B.ensure_big_onnx(path, str(tmp_path))
  with mock.patch.object(B.subprocess, 'run'):   # "succeeded" but left the pointer
    assert not B.ensure_big_onnx(path, str(tmp_path))


def test_a_missing_file_is_not_a_model(tmp_path):
  assert not B.ensure_big_onnx(tmp_path / 'absent.onnx', str(tmp_path))
