"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The guarantee core openpilot relies on: no backend can take down its caller.

modeld, manager, hardwared and the UI all reach hardware through this module,
so a backend that is absent, half-installed or simply broken must cost the
large model and nothing else. Fakes rather than hardware, so this runs anywhere.
"""
import importlib
import sys
import tempfile
import types
import unittest
from collections import namedtuple
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot import accelerators
from openpilot.sunnypilot.accelerators.base import Daemon


class FakeAccelerator:
  def __init__(self, name, present=False, ready=False, reason=None, daemon=None, raises=()):
    self.name = name
    self._present, self._ready, self._reason, self._daemon = present, ready, reason, daemon
    self._raises = raises

  def _check(self, question):
    if question in self._raises:
      raise RuntimeError(f"{self.name}.{question} exploded")

  def present(self):
    self._check('present')
    return self._present

  def ready(self):
    self._check('ready')
    return self._ready

  def unavailable_reason(self):
    self._check('unavailable_reason')
    return self._reason

  def daemon(self):
    self._check('daemon')
    return self._daemon


class AcceleratorTest(unittest.TestCase):
  def install(self, *backends):
    patcher = mock.patch.object(accelerators, '_cache', list(backends))
    patcher.start()
    self.addCleanup(patcher.stop)


class TestDiscovery(AcceleratorTest):
  def setUp(self):
    patcher = mock.patch.object(accelerators, '_cache', None)
    patcher.start()
    self.addCleanup(patcher.stop)

  def _module(self, name, attr, value):
    mod = types.ModuleType(name)
    setattr(mod, attr, value)
    sys.modules[name] = mod
    self.addCleanup(sys.modules.pop, name, None)

  def test_a_backend_this_fork_does_not_ship_is_skipped(self):
    with mock.patch.object(accelerators, '_BACKENDS', ("nonexistent.module:Thing",)):
      self.assertEqual(accelerators.backends(), [])

  def test_a_backend_that_fails_to_construct_does_not_propagate(self):
    def boom():
      raise RuntimeError("no driver")
    self._module('fake_broken_accel', 'Boom', boom)
    with mock.patch.object(accelerators, '_BACKENDS', ("fake_broken_accel:Boom",)):
      self.assertEqual(accelerators.backends(), [])

  def test_a_working_backend_is_constructed(self):
    self._module('fake_good_accel', 'Good', lambda: FakeAccelerator('good'))
    with mock.patch.object(accelerators, '_BACKENDS', ("fake_good_accel:Good",)):
      self.assertEqual([b.name for b in accelerators.backends()], ['good'])

  def test_one_broken_backend_does_not_hide_the_others(self):
    self._module('fake_good_accel', 'Good', lambda: FakeAccelerator('good'))
    with mock.patch.object(accelerators, '_BACKENDS',
                           ("nonexistent.module:Thing", "fake_good_accel:Good")):
      self.assertEqual([b.name for b in accelerators.backends()], ['good'])

  def test_the_shipped_backends_all_load(self):
    # Catches a typo in _BACKENDS, which discovery would otherwise swallow.
    for spec in accelerators._BACKENDS:
      module, _, cls = spec.partition(':')
      self.assertTrue(hasattr(__import__(module, fromlist=[cls]), cls), spec)


class TestPresent(AcceleratorTest):
  def test_nothing_attached(self):
    self.install(FakeAccelerator('a'), FakeAccelerator('b'))
    self.assertFalse(accelerators.present())

  def test_any_backend_counts(self):
    self.install(FakeAccelerator('a'), FakeAccelerator('b', present=True))
    self.assertTrue(accelerators.present())

  def test_a_raising_backend_does_not_hide_an_attached_one(self):
    self.install(FakeAccelerator('a', raises=('present',)),
                 FakeAccelerator('b', present=True))
    self.assertTrue(accelerators.present())

  def test_a_raising_backend_answers_no_rather_than_raising(self):
    self.install(FakeAccelerator('a', raises=('present',)))
    self.assertFalse(accelerators.present())


class TestActive(AcceleratorTest):
  def test_none_ready_is_the_small_model(self):
    self.install(FakeAccelerator('a', present=True), FakeAccelerator('b'))
    self.assertIsNone(accelerators.active())

  def test_first_ready_wins(self):
    self.install(FakeAccelerator('a', ready=True), FakeAccelerator('b', ready=True))
    self.assertEqual(accelerators.active().name, 'a')

  def test_order_is_priority_not_readiness_order(self):
    # Local hardware is listed first so a board beats a link when both are up.
    self.install(FakeAccelerator('a'), FakeAccelerator('b', ready=True))
    self.assertEqual(accelerators.active().name, 'b')

  def test_a_raising_backend_is_skipped_not_selected(self):
    self.install(FakeAccelerator('a', raises=('ready',)), FakeAccelerator('b', ready=True))
    self.assertEqual(accelerators.active().name, 'b')


class TestUnavailableReason(AcceleratorTest):
  def test_silence_when_there_is_nothing_to_say(self):
    self.install(FakeAccelerator('a'), FakeAccelerator('b'))
    self.assertIsNone(accelerators.unavailable_reason())

  def test_the_first_complaint_is_reported(self):
    self.install(FakeAccelerator('a'), FakeAccelerator('b', reason='no gadget'),
                 FakeAccelerator('c', reason='also broken'))
    self.assertEqual(accelerators.unavailable_reason(), 'no gadget')

  def test_a_raising_backend_does_not_break_the_alert_pass(self):
    self.install(FakeAccelerator('a', raises=('unavailable_reason',)),
                 FakeAccelerator('b', reason='no gadget'))
    self.assertEqual(accelerators.unavailable_reason(), 'no gadget')


class TestDaemons(AcceleratorTest):
  def test_backends_without_a_daemon_contribute_nothing(self):
    self.install(FakeAccelerator('a'), FakeAccelerator('b'))
    self.assertEqual(accelerators.daemons(), [])

  def test_a_declared_daemon_is_collected(self):
    d = Daemon('somed', 'some.module', lambda *a: True)
    self.install(FakeAccelerator('a'), FakeAccelerator('b', daemon=d))
    self.assertEqual(accelerators.daemons(), [d])

  def test_a_raising_backend_does_not_stop_manager_starting(self):
    d = Daemon('somed', 'some.module', lambda *a: True)
    self.install(FakeAccelerator('a', raises=('daemon',)), FakeAccelerator('b', daemon=d))
    self.assertEqual(accelerators.daemons(), [d])

  def test_declared_daemons_are_importable(self):
    # A daemon manager cannot import would fail at the onroad transition, long
    # after anything would notice here.
    for d in accelerators.daemons():
      __import__(d.module)


class TestProgress(unittest.TestCase):
  def test_a_missing_param_is_no_progress(self):
    with mock.patch.object(accelerators, 'Params') as params:
      params.return_value.get.return_value = None
      self.assertIsNone(accelerators.progress())

  def test_a_dict_comes_through(self):
    payload = {'stage': 'build', 'frac': 0.5, 'msg': ''}
    with mock.patch.object(accelerators, 'Params') as params:
      params.return_value.get.return_value = payload
      self.assertEqual(accelerators.progress(), payload)

  def test_a_non_dict_is_ignored(self):
    with mock.patch.object(accelerators, 'Params') as params:
      params.return_value.get.return_value = "build 50%"
      self.assertIsNone(accelerators.progress())

  def test_an_unknown_key_does_not_take_down_the_ui(self):
    # A params library older than this key raises rather than returning None.
    with mock.patch.object(accelerators, 'Params') as params:
      params.return_value.get.side_effect = RuntimeError("UnknownKeyName")
      self.assertIsNone(accelerators.progress())

  def test_reporting_never_raises(self):
    # Called from except handlers in the daemons.
    with mock.patch.object(accelerators, 'Params') as params:
      params.return_value.put.side_effect = RuntimeError("params gone")
      accelerators.report_progress('build', 0.5)
      params.return_value.remove.side_effect = RuntimeError("params gone")
      accelerators.clear_progress()



class TestShutdown(AcceleratorTest):
  def test_every_backend_with_a_say_is_told_and_the_rest_are_skipped(self):
    told = []

    class WithShutdown(FakeAccelerator):
      def shutdown(self, reason):
        told.append((self.name, reason))

    class Exploding(FakeAccelerator):
      def shutdown(self, reason):
        raise RuntimeError('no')

    self.install(FakeAccelerator('mute'), Exploding('loud'), WithShutdown('jetlink'))
    accelerators.shutdown('car battery')
    assert told == [('jetlink', 'car battery')]


class TestChestnutReadyFallback(unittest.TestCase):
  """chestnut.py must survive modeld.helpers losing chestnut_ready.

  comma added it in #38742 and reverted it the next day in #38760. A
  module-scope import of a name upstream has taken away raises ImportError,
  which backends() swallows as "a fork that does not ship this backend" - so a
  real chestnut would quietly stop running the large model, with nothing in the
  log to say why. The backend carries its own copy for exactly that window.
  """

  State = namedtuple('State', 'supplyVoltage supplyFault pcieLtssm')

  def _reload_without_chestnut_ready(self):
    from openpilot.selfdrive.modeld import helpers
    from openpilot.sunnypilot.accelerators import chestnut

    self.addCleanup(importlib.reload, chestnut)
    saved = helpers.chestnut_ready
    self.addCleanup(setattr, helpers, 'chestnut_ready', saved)
    del helpers.chestnut_ready
    return importlib.reload(chestnut)

  def test_the_backend_still_loads(self):
    mod = self._reload_without_chestnut_ready()
    self.assertTrue(hasattr(mod, 'ChestnutAccelerator'))
    self.assertEqual(mod.ChestnutAccelerator().name, 'chestnut')

  def test_the_local_check_agrees_with_the_one_upstream_removed(self):
    mod = self._reload_without_chestnut_ready()
    good = self.State(supplyVoltage=5000, supplyFault=False, pcieLtssm=0x78)
    self.assertTrue(mod.chestnut_ready(good))
    for bad in (good._replace(supplyVoltage=4999), good._replace(supplyFault=True),
                good._replace(pcieLtssm=0x00)):
      self.assertFalse(mod.chestnut_ready(bad), bad)


class TestWarpCache(unittest.TestCase):
  """The warp is a build product now (accelerators/SConscript), not something
  jetlinkd compiles against a staleness key of its own."""

  GEOMETRY = (1928, 1208, 512, 256)

  def test_the_cache_lives_in_the_tree(self):
    # scons has to be able to write it; the updater deletes it exactly as it
    # deletes upstream's pkls and the build that follows puts it back. It used
    # to sit under Paths.comma_home(), which on AGNOS is a tmpfs overlay that
    # loses it every boot, and which OPENPILOT_PREFIX moves out from under a
    # replay.
    from openpilot.common.basedir import BASEDIR
    from openpilot.sunnypilot.accelerators.jetlink import warp_cache
    self.assertEqual(warp_cache.CACHE_DIR.name, 'models')
    self.assertTrue(warp_cache.CACHE_DIR.is_relative_to(Path(BASEDIR).resolve()))

  def test_a_pickle_with_no_sidecar_counts_as_cached(self):
    # The sidecar and its build key are gone; scons owns staleness. A warp the
    # build wrote and never annotated is the normal case now.
    from openpilot.sunnypilot.accelerators.jetlink import warp_cache
    with tempfile.TemporaryDirectory() as d:
      with mock.patch.object(warp_cache, 'CACHE_DIR', Path(d)):
        self.assertFalse(warp_cache.is_cached(*self.GEOMETRY))
        warp_cache.warp_path(*self.GEOMETRY).touch()
        self.assertTrue(warp_cache.is_cached(*self.GEOMETRY))

  def test_the_geometry_the_device_asks_for_names_the_file(self):
    from openpilot.sunnypilot.accelerators.jetlink import warp_cache
    cam_w, cam_h, model_w, model_h = warp_cache.device_geometry()
    self.assertEqual(warp_cache.warp_path(cam_w, cam_h, model_w, model_h).name,
                     f'warp_{cam_w}x{cam_h}_{model_w}x{model_h}_tinygrad.pkl')


class TestJetlinkdWarpFallback(unittest.TestCase):
  """jetlinkd only builds a warp that is missing outright."""

  def _daemon(self):
    from openpilot.sunnypilot.accelerators.jetlink import jetlinkd
    d = jetlinkd.Jetlinkd.__new__(jetlinkd.Jetlinkd)   # no __init__: it wants Params
    d.warp_built, d.warp_thread = False, None
    return jetlinkd, d

  def test_a_warp_the_build_made_is_left_alone(self):
    # Reporting progress before checking put a "compiling the camera warp"
    # through the UI on every start for a warp that was already there.
    jetlinkd, d = self._daemon()
    with mock.patch.object(jetlinkd.warp_cache, 'is_cached', return_value=True), \
         mock.patch.object(jetlinkd.warp_cache, 'ensure') as ensure, \
         mock.patch.object(jetlinkd.accelerators, 'report_progress') as report:
      d.build_warp()
    report.assert_not_called()
    ensure.assert_not_called()
    self.assertIsNone(d.warp_thread)

  def test_a_missing_warp_is_still_built(self):
    jetlinkd, d = self._daemon()
    with mock.patch.object(jetlinkd.warp_cache, 'is_cached', return_value=False), \
         mock.patch.object(jetlinkd.warp_cache, 'ensure', return_value=True) as ensure, \
         mock.patch.object(jetlinkd.accelerators, 'report_progress') as report, \
         mock.patch.object(jetlinkd.accelerators, 'clear_progress'):
      d.build_warp()
      self.assertIsNotNone(d.warp_thread)
      d.warp_thread.join(30)
    self.assertFalse(d.warp_thread.is_alive())
    ensure.assert_called_once()
    self.assertEqual(report.call_args.args[0], 'warp')

if __name__ == "__main__":
  unittest.main()
