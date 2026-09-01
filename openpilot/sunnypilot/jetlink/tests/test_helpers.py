"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.jetlink import helpers


class TestGadgetStatus(unittest.TestCase):
  """The gadget is set up at boot, by root, from launch_chffrplus.sh. This file
  is the only way the reason for a failure reaches anything a user can see."""

  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.status = Path(self.tmp) / 'jetlink-gadget'
    patcher = mock.patch.object(helpers, 'GADGET_STATUS', self.status)
    self.addCleanup(patcher.stop)
    patcher.start()

  def test_missing_file_is_not_an_error(self):
    # A build that never ran the setup at all reads the same as not installed.
    assert helpers.gadget_error() is None

  def test_ok_is_not_an_error(self):
    self.status.write_text('ok\n')
    assert helpers.gadget_error() is None

  def test_empty_is_not_an_error(self):
    self.status.write_text('')
    assert helpers.gadget_error() is None

  def test_reason_is_unwrapped(self):
    self.status.write_text('error: kernel has no USB gadget support\n')
    assert helpers.gadget_error() == 'kernel has no USB gadget support'

  def test_bare_reason_survives(self):
    self.status.write_text('something went wrong')
    assert helpers.gadget_error() == 'something went wrong'

  def test_unreadable_status_is_not_an_error(self):
    # Path.exists() and read_text() raise rather than return on a root-only
    # path; an availability check must never take a process down over one.
    with mock.patch.object(Path, 'read_text', side_effect=PermissionError):
      assert helpers.gadget_error() is None


class TestGadgetAlert(unittest.TestCase):
  """Only complain to someone who asked for the link. On auto, a device that
  cannot present the gadget should simply not offer the feature."""

  def alert_with(self, opted_in: bool, reason: str | None):
    with mock.patch.object(helpers, 'opted_in', return_value=opted_in), \
         mock.patch.object(helpers, 'gadget_error', return_value=reason):
      return helpers.gadget_alert()

  def test_silent_on_auto(self):
    assert self.alert_with(False, 'kernel has no USB gadget support') is None

  def test_speaks_up_when_switched_on(self):
    assert self.alert_with(True, 'kernel has no USB gadget support') == 'kernel has no USB gadget support'

  def test_nothing_to_say_when_healthy(self):
    assert self.alert_with(True, None) is None


class TestUnchunkedSuffix(unittest.TestCase):
  def test_suffix_is_distinct_from_openpilots(self):
    """openpilot's own '.unchunked' files are deleted by an atexit handler.
    Ours must not collide with that or a provision loses its model mid-upload."""
    assert helpers.UNCHUNKED_SUFFIX != '.unchunked'
    assert not helpers.UNCHUNKED_SUFFIX.endswith('.unchunked')


class TestActiveModelPath(unittest.TestCase):
  def setUp(self):
    self.root = tempfile.mkdtemp()
    patcher = mock.patch('openpilot.sunnypilot.jetlink.helpers.Paths')
    self.addCleanup(patcher.stop)
    patcher.start().model_root.return_value = self.root

  def bundle_with(self, *file_names):
    models = []
    for name in file_names:
      artifact = mock.Mock()
      artifact.fileName = name
      models.append(mock.Mock(artifact=artifact))
    return mock.Mock(models=models)

  def test_no_bundle_is_no_path(self):
    with mock.patch.object(helpers, 'active_bundle', return_value=None):
      assert helpers.active_model_path() is None

  def test_finds_a_plain_onnx(self):
    onnx = Path(self.root) / 'big.onnx'
    onnx.write_bytes(b'\0' * 2_000_000)
    with mock.patch.object(helpers, 'active_bundle', return_value=self.bundle_with('big.onnx')):
      assert helpers.active_model_path() == onnx

  def test_ignores_a_truncated_download(self):
    (Path(self.root) / 'big.onnx').write_bytes(b'\0' * 16)
    with mock.patch.object(helpers, 'active_bundle', return_value=self.bundle_with('big.onnx')), \
         mock.patch('openpilot.selfdrive.modeld.helpers.MODELS_DIR', Path(self.root)):
      assert helpers.active_model_path() is None

  def test_ignores_non_onnx_artifacts(self):
    (Path(self.root) / 'small.pkl').write_bytes(b'\0' * 2_000_000)
    with mock.patch.object(helpers, 'active_bundle', return_value=self.bundle_with('small.pkl')), \
         mock.patch('openpilot.selfdrive.modeld.helpers.MODELS_DIR', Path(self.root)):
      assert helpers.active_model_path() is None

  def test_cleanup_keeps_the_one_in_use(self):
    keep = Path(self.root) / f'a.onnx{helpers.UNCHUNKED_SUFFIX}'
    drop = Path(self.root) / f'b.onnx{helpers.UNCHUNKED_SUFFIX}'
    keep.write_bytes(b'k')
    drop.write_bytes(b'd')
    helpers.cleanup_unchunked(keep=keep)
    assert keep.is_file()
    assert not drop.exists()


if __name__ == '__main__':
  unittest.main()


class TestShippedModelPath(unittest.TestCase):
  """The pointer decides which model this branch means. sunnypilot substitutes
  its own big model for comma's, so a file of the wrong size under that name is
  a model fetched for a different branch, not this one."""

  def setUp(self):
    self.root = tempfile.mkdtemp()
    self.tree = Path(tempfile.mkdtemp())
    paths = mock.patch('openpilot.sunnypilot.jetlink.helpers.Paths')
    self.addCleanup(paths.stop)
    paths.start().model_root.return_value = self.root
    pointer = mock.patch.object(helpers, 'big_model_pointer', return_value=self.tree / 'big.onnx')
    self.addCleanup(pointer.stop)
    pointer.start()

  def write_pointer(self, size: int) -> None:
    (self.tree / 'big.onnx').write_text(
      f"version https://git-lfs.github.com/spec/v1\noid sha256:{'a' * 64}\nsize {size}\n")

  def fetched(self, size: int) -> Path:
    path = Path(self.root) / helpers.BIG_MODEL_NAME
    path.write_bytes(b'\0' * size)
    return path

  def test_nothing_fetched_yet(self):
    self.write_pointer(4096)
    assert helpers.shipped_model_path() is None

  def test_the_pinned_object_is_accepted(self):
    self.write_pointer(4096)
    fetched = self.fetched(4096)
    assert helpers.shipped_model_path() == fetched

  def test_a_different_branch_model_is_rejected(self):
    self.write_pointer(4096)
    self.fetched(2048)
    assert helpers.shipped_model_path() is None

  def test_an_unexcluded_object_is_used_in_place(self):
    # A checkout that did fetch the object has the real thing in the worktree.
    real = self.tree / 'big.onnx'
    real.write_bytes(b'\0' * 2_000_000)
    assert helpers.shipped_model_path() == real
