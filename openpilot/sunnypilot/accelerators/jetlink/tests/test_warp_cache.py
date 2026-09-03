"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

When the cached warp JIT is trusted, and when it is not.

The compile itself needs a GPU and is not exercised here; what is, is every way
the cache can be wrong. A stale pickle is the dangerous case: a TinyJit from
another tinygrad either fails to load or loads into something that does not
compute the warp, and modeld would carry that to the car. So the rule is that
anything short of an exact build match is a miss, and a miss costs the large
model rather than producing a wrong one.
"""
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.accelerators.jetlink import warp_cache

GEOM = (1928, 1208, 512, 256)


class WarpCacheTest(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    patcher = mock.patch.object(warp_cache, 'CACHE_DIR', Path(self.tmp.name))
    patcher.start()
    self.addCleanup(patcher.stop)
    build = mock.patch.object(warp_cache, '_build_key', return_value='commit-a')
    build.start()
    self.addCleanup(build.stop)

  def write(self, geom=GEOM, build='commit-a', body=b'pickle'):
    pkl = warp_cache.warp_path(*geom)
    pkl.parent.mkdir(parents=True, exist_ok=True)
    pkl.write_bytes(body)
    pkl.with_suffix('.json').write_text(json.dumps({'build': build}))
    return pkl


class TestValidity(WarpCacheTest):
  def test_a_warp_from_this_build_is_used(self):
    self.write()
    self.assertTrue(warp_cache.is_cached(*GEOM))

  def test_nothing_cached_is_a_miss(self):
    self.assertFalse(warp_cache.is_cached(*GEOM))

  def test_a_warp_from_another_build_is_a_miss(self):
    """tinygrad is a submodule the commit pins, so another commit is another
    tinygrad, and its pickled JIT cannot be trusted to be loadable."""
    self.write(build='commit-b')
    self.assertFalse(warp_cache.is_cached(*GEOM))

  def test_a_pickle_with_no_sidecar_is_a_miss(self):
    """Half a write, or a file left by hand. Nothing says what made it."""
    pkl = warp_cache.warp_path(*GEOM)
    pkl.parent.mkdir(parents=True, exist_ok=True)
    pkl.write_bytes(b'pickle')
    self.assertFalse(warp_cache.is_cached(*GEOM))

  def test_a_corrupt_sidecar_is_a_miss_not_a_raise(self):
    pkl = self.write()
    pkl.with_suffix('.json').write_text('{ not json')
    self.assertFalse(warp_cache.is_cached(*GEOM))

  def test_another_camera_does_not_answer_for_this_one(self):
    """A device that changed camera, or a cache copied between devices. The
    warp is baked against the frame geometry, so the wrong one is silently
    wrong rather than an error."""
    self.write(geom=(1344, 760, 512, 256))
    self.assertFalse(warp_cache.is_cached(*GEOM))

  def test_another_model_input_size_does_not_answer_either(self):
    self.write(geom=(1928, 1208, 256, 128))
    self.assertFalse(warp_cache.is_cached(*GEOM))


class TestLoad(WarpCacheTest):
  def test_a_miss_raises_rather_than_returning_none(self):
    """modeld's big-model load is wrapped in the one-way fallback to the small
    model. Raising lands there; returning None would reach the car."""
    with self.assertRaises(RuntimeError):
      warp_cache.load_warp(*GEOM)

  def test_a_stale_warp_raises_even_though_the_file_is_there(self):
    self.write(build='commit-b')
    with self.assertRaises(RuntimeError):
      warp_cache.load_warp(*GEOM)


class TestEnsure(WarpCacheTest):
  def test_a_cached_warp_is_not_rebuilt(self):
    self.write()
    with mock.patch.object(warp_cache, 'compile_warp') as compile_warp:
      self.assertTrue(warp_cache.ensure(*GEOM))
    compile_warp.assert_not_called()

  def test_a_failed_compile_is_reported_not_raised(self):
    """jetlinkd's loop must survive this: no warp costs the large model, and
    taking the daemon down with it would also drop the USB gadget."""
    with mock.patch.object(warp_cache, 'compile_warp', side_effect=RuntimeError("no gpu")):
      self.assertFalse(warp_cache.ensure(*GEOM))

  def test_a_successful_build_prunes_the_others(self):
    stale = self.write(geom=(1344, 760, 512, 256))
    fresh = warp_cache.warp_path(*GEOM)
    with mock.patch.object(warp_cache, 'compile_warp', side_effect=lambda *a: self.write()):
      self.assertTrue(warp_cache.ensure(*GEOM))
    self.assertTrue(fresh.is_file())
    self.assertFalse(stale.is_file())
    self.assertFalse(stale.with_suffix('.json').is_file())


class TestGeometry(WarpCacheTest):
  def test_mici_and_tici_want_different_warps(self):
    """The same split SConscript makes, so jetlinkd builds what modeld asks
    for. If these ever drift, load_warp misses and the drive is small-model."""
    with mock.patch("openpilot.common.hardware.HARDWARE.get_device_type", return_value="mici"):
      mici = warp_cache.device_geometry()
    with mock.patch("openpilot.common.hardware.HARDWARE.get_device_type", return_value="tici"):
      tici = warp_cache.device_geometry()
    self.assertNotEqual(mici[:2], tici[:2])
    # both warp to MEDMODEL_INPUT_SIZE, which is what the model input needs
    self.assertEqual(mici[2:], tici[2:])
    self.assertEqual(tici[2:], (512, 256))


if __name__ == "__main__":
  unittest.main()
