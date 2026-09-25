"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import tempfile
import unittest
import urllib.request
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.accelerators.jetlink import gadget, helpers


class TestGadgetStatus(unittest.TestCase):
  """The gadget is set up at boot, by root, from launch_chffrplus.sh. This file
  is the only way the reason for a failure reaches anything a user can see."""

  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.status = Path(self.tmp) / 'jetlink-gadget'
    patcher = mock.patch.object(gadget, 'GADGET_STATUS', self.status)
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


class TestGadgetSetup(unittest.TestCase):
  """jetlinkd creates the gadget when the link was turned on after boot."""

  def setUp(self):
    self.tmp = Path(tempfile.mkdtemp())
    script = self.tmp / 'jetlink_repo' / 'scripts' / 'setup_gadget.sh'
    script.parent.mkdir(parents=True)
    script.write_text('#!/bin/sh\n')
    self.script = script
    for name, value in (('repo_root', mock.Mock(return_value=self.tmp)), ('AGNOS', True)):
      p = mock.patch.object(gadget, name, value)
      self.addCleanup(p.stop)
      p.start()

  def test_the_package_is_installed_when_the_submodule_is_checked_out(self):
    assert not helpers.package_installed()
    pkg = self.tmp / 'jetlink_repo' / 'jetlink'
    pkg.mkdir()
    (pkg / '__init__.py').write_text('')
    assert helpers.package_installed()

  def test_only_agnos_with_the_script_can_set_one_up(self):
    assert helpers.can_setup_gadget()
    with mock.patch.object(gadget, 'AGNOS', False):
      assert not helpers.can_setup_gadget()
    self.script.unlink()
    assert not helpers.can_setup_gadget()

  def test_setup_runs_the_boot_script_as_root_and_reports_the_result(self):
    with mock.patch.object(gadget.subprocess, 'run') as run, \
         mock.patch.object(gadget, 'link_configured', return_value=True):
      assert helpers.setup_gadget()
    (argv,), kwargs = run.call_args
    assert argv[:3] == ['sudo', '-n', 'bash'] and argv[3] == str(self.script)
    assert kwargs['check'] and kwargs['timeout'] == helpers.GADGET_SETUP_TIMEOUT

  def test_a_failed_script_is_a_false_not_a_raise(self):
    # the script has already written the reason to the status file
    with mock.patch.object(gadget.subprocess, 'run', side_effect=gadget.subprocess.CalledProcessError(1, 'bash')), \
         mock.patch.object(helpers.cloudlog, 'exception') as log:
      assert not helpers.setup_gadget()
    assert log.call_count == 1


class TestGadgetAlert(unittest.TestCase):
  """Only complain to someone who asked for the link. With it off, a device
  that cannot present the gadget should simply not offer the feature."""

  def alert_with(self, enabled: bool, reason: str | None):
    with mock.patch.object(gadget, 'enabled', return_value=enabled), \
         mock.patch.object(gadget, 'gadget_error', return_value=reason):
      return helpers.gadget_alert()

  def test_silent_when_off(self):
    assert self.alert_with(False, 'kernel has no USB gadget support') is None

  def test_speaks_up_when_switched_on(self):
    assert self.alert_with(True, 'kernel has no USB gadget support') == 'kernel has no USB gadget support'

  def test_nothing_to_say_when_healthy(self):
    assert self.alert_with(True, None) is None


class TestDormant(unittest.TestCase):
  def setUp(self):
    self.tmp = Path(tempfile.mkdtemp())
    for name in ('DORMANT', 'SHUTDOWN_REQUEST'):
      # gadget owns the paths and reads them from its own namespace
      patcher = mock.patch.object(gadget, name, self.tmp / name.lower())
      self.addCleanup(patcher.stop)
      patcher.start()

  def test_marker_from_a_live_process_counts(self):
    helpers.set_dormant(True)
    assert helpers.dormant()
    helpers.set_dormant(False)
    assert not helpers.dormant()

  def test_marker_from_a_dead_process_is_a_leftover(self):
    helpers.DORMANT.write_text('4194304')  # above pid_max
    assert not helpers.dormant()

  def test_garbage_is_not_dormant(self):
    helpers.DORMANT.write_text('not a pid')
    assert not helpers.dormant()

  def test_dormant_counts_as_present_without_a_host(self):
    with mock.patch.object(gadget, 'link_endpoint', return_value=None), \
         mock.patch.object(gadget, 'host_attached', return_value=False), \
         mock.patch.object(gadget, 'CC_ORIENTATION', self.tmp / 'cc'):
      (self.tmp / 'cc').write_text('1')
      helpers._last_configured = 0.0
      assert not helpers.gadget_present()
      helpers.set_dormant(True)
      assert helpers.gadget_present()
      (self.tmp / 'cc').write_text('0')
      assert not helpers.gadget_present()

  def test_shutdown_request_round_trip(self):
    assert helpers.pending_shutdown() is None
    assert helpers.request_shutdown('car battery')
    assert helpers.pending_shutdown() == 'car battery'
    helpers.finish_shutdown()
    assert helpers.pending_shutdown() is None

  def test_await_shutdown_gives_up_and_cleans_up(self):
    helpers.request_shutdown('car battery')
    assert not helpers.await_shutdown(0.3)
    assert helpers.pending_shutdown() is None

  def test_await_shutdown_returns_when_taken(self):
    helpers.request_shutdown('car battery')
    helpers.finish_shutdown()
    assert helpers.await_shutdown(0.3)


def bundle(ref: str, name: str, index: int = 0, version=19) -> dict:
  """A bundle as the catalog JSON carries it."""
  return {'ref': ref, 'display_name': name, 'index': index, 'minimum_selector_version': str(version)}


REF_A, REF_B, REF_C = 'a' * 40, 'b' * 40, 'c' * 40
POINTERS = {REF_A: {'oid': '1' * 64, 'size': 766_000_000},
            REF_B: {'oid': '2' * 64, 'size': 1_757_000_000}}


def catalog_param(*bundles):
  params = mock.patch.object(helpers, 'params')
  params.start().return_value.get.return_value = {'bundles': list(bundles)}
  return params


class TestCatalog(unittest.TestCase):
  """The list is sunnypilot's big-model catalog, read as the model manager cached it."""

  def setUp(self):
    self.addCleanup(mock.patch.stopall)

  def test_newest_first_with_names(self):
    catalog_param(bundle(REF_A, 'Alpha (September 04, 2026)', 3), bundle(REF_B, 'Beta', 9))
    self.assertEqual(helpers.catalog(), [{'name': 'Beta', 'ref': REF_B}, {'name': 'Alpha (September 04, 2026)', 'ref': REF_A}])

  def test_only_commits_of_this_selector_version(self):
    # a bundle without a comma commit has no ONNX to find; one for another
    # selector version is one the model manager itself would not list
    catalog_param(bundle('not-a-commit', 'Odd', 5), bundle(REF_A, 'Alpha', 1), {'display_name': 'Blank'},
                  bundle(REF_C, 'Gamma', 7, version=18))
    self.assertEqual([b['ref'] for b in helpers.catalog()], [REF_A])

  def test_no_catalog_yet_is_empty(self):
    catalog_param()
    self.assertEqual(helpers.catalog(), [])

  def test_an_unreadable_catalog_is_empty_not_an_error(self):
    # read from the UI's param thread, where an exception takes the panel down
    params = mock.patch.object(helpers, 'params').start()
    params.return_value.get.side_effect = RuntimeError('no params')
    self.assertEqual(helpers.catalog(), [])


class TestModelIndex(unittest.TestCase):
  """Every catalog model, with the ONNX behind it once that has been looked up."""

  def setUp(self):
    helpers._index_cache = None
    self.addCleanup(setattr, helpers, '_index_cache', None)

  def index_with(self, bundles, pointers=POINTERS):
    with mock.patch.object(helpers, 'catalog', return_value=bundles), \
         mock.patch.object(helpers, 'pointers', return_value=pointers):
      return helpers.model_index()

  def test_a_resolved_model_carries_its_identity(self):
    (entry,) = self.index_with([{'name': 'Alpha', 'ref': REF_A}])
    self.assertEqual(entry, {'name': 'Alpha', 'ref': REF_A, 'oid': '1' * 64, 'size': 766_000_000})

  def test_an_unresolved_model_is_still_listed(self):
    # the pointer is fetched when the model is first asked for
    (entry,) = self.index_with([{'name': 'Gamma', 'ref': REF_C}])
    self.assertEqual((entry['name'], entry['oid'], entry['size']), ('Gamma', None, None))

  def test_a_second_read_within_the_ttl_costs_nothing(self):
    # the UI names the active model every frame
    with mock.patch.object(helpers, 'catalog', return_value=[]) as read, mock.patch.object(helpers, 'pointers', return_value={}):
      first = helpers.model_index()
      self.assertIs(helpers.model_index(), first)
    self.assertEqual(read.call_count, 1)


class TestResolvePointer(unittest.TestCase):
  """The pointer at a commit is the oid and size the Jetson is asked for,
  fetched the first time a model is asked for and kept for good."""

  POINTER = f"version https://git-lfs.github.com/spec/v1\noid sha256:{'3' * 64}\nsize 766040736\n"

  def setUp(self):
    self.params = mock.patch.object(helpers, 'params').start()
    self.addCleanup(mock.patch.stopall)
    helpers._index_cache = (float('inf'), [])   # a stale index must be dropped on a hit
    self.addCleanup(setattr, helpers, '_index_cache', None)

  def response(self, body: bytes):
    r = mock.MagicMock()
    r.__enter__.return_value = r
    r.read.return_value = body
    return r

  def test_fetches_once_and_records_it(self):
    from jetlink.registry.lfs import POINTER_URL
    with mock.patch.object(helpers, '_get', return_value=dict(POINTERS)), \
         mock.patch.object(urllib.request, 'urlopen', return_value=self.response(self.POINTER.encode())) as urlopen:
      self.assertEqual(helpers.resolve_pointer(REF_C), ('3' * 64, 766040736))
    self.assertEqual(urlopen.call_args.args[0], POINTER_URL.format(ref=REF_C))
    written = self.params.return_value.put.call_args.args[1]
    self.assertEqual(written[REF_C], {'oid': '3' * 64, 'size': 766040736})
    self.assertEqual(written[REF_A], POINTERS[REF_A])
    self.assertIsNone(helpers._index_cache)

  def test_a_known_pointer_needs_no_fetch(self):
    with mock.patch.object(helpers, '_get', return_value=dict(POINTERS)), \
         mock.patch.object(urllib.request, 'urlopen') as urlopen:
      self.assertEqual(helpers.resolve_pointer(REF_A), ('1' * 64, 766_000_000))
    urlopen.assert_not_called()

  def test_a_miss_raises_and_records_nothing(self):
    from jetlink.registry.catalog import RegistryError
    for failure in ({'side_effect': OSError('offline')}, {'return_value': self.response(b'<html>not found</html>')}):
      with self.subTest(failure), mock.patch.object(helpers, '_get', return_value={}), \
           mock.patch.object(urllib.request, 'urlopen', **failure), self.assertRaises(RegistryError):
        helpers.resolve_pointer(REF_C)
    self.params.return_value.put.assert_not_called()

  def test_the_lookup_is_the_registry_s(self):
    """One resolver for both ends; the precompiled-pkl commits are tested there."""
    from jetlink.registry.lfs import Pointer
    with mock.patch.object(helpers, '_get', return_value={}), \
         mock.patch('jetlink.registry.lfs.fetch_pointer', return_value=Pointer('4' * 64, 766354845)) as fetch:
      self.assertEqual(helpers.resolve_pointer(REF_C), ('4' * 64, 766354845))
    fetch.assert_called_once_with(REF_C, timeout=helpers.POINTER_TIMEOUT)


class TestSelectedModel(unittest.TestCase):
  """The pick is the model manager's big-model slot, the same one a chestnut runs from."""

  INDEX = [
    {'name': 'Alpha', 'ref': REF_A, 'oid': 'a' * 64, 'size': 10},
    {'name': 'Beta', 'ref': REF_B, 'oid': 'b' * 64, 'size': 20},
  ]

  def select_with(self, slot_ref, default=REF_B):
    with mock.patch.object(helpers, 'model_index', return_value=self.INDEX), \
         mock.patch.object(helpers, 'DEFAULT_BIG_MODEL_REF', default), \
         mock.patch.object(helpers, 'selected_ref', return_value=slot_ref):
      return helpers.selected_model()

  def test_an_empty_slot_takes_the_forks_default_big_model(self):
    assert self.select_with(None)['name'] == 'Beta'

  def test_the_slots_ref_selects_it(self):
    assert self.select_with(REF_A)['name'] == 'Alpha'

  def test_a_default_not_in_the_catalog_falls_to_the_newest(self):
    assert self.select_with(None, default='f' * 40)['name'] == 'Alpha'

  def test_a_ref_the_catalog_dropped_falls_back(self):
    # leaving the device with no model at all would be worse than quietly using the default
    assert self.select_with('9' * 40)['name'] == 'Beta'

  def test_an_empty_index_is_no_model(self):
    with mock.patch.object(helpers, 'model_index', return_value=[]):
      assert helpers.selected_model() is None


class TestSelectedRef(unittest.TestCase):
  def setUp(self):
    helpers._slot_cache = None
    self.addCleanup(setattr, helpers, '_slot_cache', None)

  def read_with(self, slot):
    helpers._slot_cache = None
    with mock.patch.object(helpers, '_get', return_value=slot):
      return helpers.selected_ref()

  def test_a_second_read_within_the_ttl_costs_nothing(self):
    # the UI names the active model every frame
    with mock.patch.object(helpers, '_get', return_value={'ref': REF_A}) as read:
      helpers._slot_cache = None
      assert helpers.selected_ref() == REF_A
      assert helpers.selected_ref() == REF_A
    assert read.call_count == 1

  def test_reads_the_slots_ref(self):
    assert self.read_with({'ref': REF_A, 'displayName': 'Alpha'}) == REF_A

  def test_anything_else_is_no_pick(self):
    for slot in (None, {}, {'ref': ''}, {'ref': 7}, 'junk'):
      assert self.read_with(slot) is None, slot


class TestMigrateSelection(unittest.TestCase):
  """JetlinkModel was the accelerator's own pick. It moves into the big-model slot
  once; a name from before the catalog maps to the engine that is ready, so a
  160 s rebuild is not the price of the rename."""

  CATALOG = [{'name': 'Alpha', 'ref': REF_A}, {'name': 'Beta', 'ref': REF_B}]

  def migrate(self, legacy, ready=None, slot_ref=None, resolve=None, listed=True):
    values = {helpers.P_MODEL_LEGACY: legacy, helpers.P_READY: ready}
    params = mock.patch.object(helpers, 'params').start()
    self.addCleanup(mock.patch.stopall)
    self.stored = mock.patch.object(helpers, '_store_slot', **({'side_effect': listed} if isinstance(listed, Exception) else {'return_value': listed})).start()
    with mock.patch.object(helpers, '_get', side_effect=lambda k, d=None: values.get(k, d)), \
         mock.patch.object(helpers, 'selected_ref', return_value=slot_ref), \
         mock.patch.object(helpers, 'catalog', return_value=self.CATALOG), \
         mock.patch.object(helpers, 'resolve_pointer', side_effect=resolve or (lambda ref: self.fail("nothing to look up"))):
      helpers.migrate_selection()
    return params.return_value

  def test_a_ref_becomes_the_slot(self):
    params = self.migrate(REF_A)
    self.stored.assert_called_once_with(params, REF_A)
    params.remove.assert_called_once_with(helpers.P_MODEL_LEGACY)

  def test_an_old_name_becomes_the_slot_of_the_model_that_is_provisioned(self):
    params = self.migrate('Beta v6', ready='b' * 64, resolve=lambda ref: ({REF_A: 'a' * 64, REF_B: 'b' * 64}[ref], 1))
    self.stored.assert_called_once_with(params, REF_B)
    params.remove.assert_called_once_with(helpers.P_MODEL_LEGACY)

  def test_an_old_name_with_nothing_provisioned_is_dropped(self):
    params = self.migrate('Beta v6')
    self.stored.assert_not_called()
    params.remove.assert_called_once_with(helpers.P_MODEL_LEGACY)

  def test_a_ref_the_catalog_does_not_list_is_dropped(self):
    params = self.migrate(REF_C, listed=False)
    params.remove.assert_called_once_with(helpers.P_MODEL_LEGACY)

  def test_no_catalog_yet_is_left_for_the_next_run(self):
    params = self.migrate(REF_A, listed=LookupError('no catalog'))
    params.remove.assert_not_called()

  def test_a_slot_already_picked_wins(self):
    params = self.migrate(REF_A, slot_ref=REF_B)
    self.stored.assert_not_called()
    params.remove.assert_called_once_with(helpers.P_MODEL_LEGACY)

  def test_nothing_to_migrate_touches_nothing(self):
    params = self.migrate(None)
    params.remove.assert_not_called()

  def test_offline_is_left_for_the_next_run(self):
    params = self.migrate('Beta v6', ready='b' * 64, resolve=OSError('offline'))
    self.stored.assert_not_called()
    params.remove.assert_not_called()


class TestSelectedModelReadiness(unittest.TestCase):
  def test_old_cached_engine_is_not_the_new_selection(self):
    # What the UI calls compiled. The join does not stop here: an engine the
    # Jetson has not got is built onroad, see backend._open_link.
    from types import SimpleNamespace
    from openpilot.sunnypilot.accelerators.jetlink import backend

    with mock.patch.object(helpers, 'enabled', return_value=True), \
         mock.patch.object(helpers, 'engine_ready_for', return_value=True), \
         mock.patch.object(backend.spec_cache, 'load', return_value=SimpleNamespace(sha256='a' * 64)), \
         mock.patch.object(helpers, 'selected_model', return_value={'oid': 'b' * 64}) as selected:
      self.assertFalse(backend.ready())
      selected.return_value = {'oid': 'a' * 64}
      self.assertTrue(backend.ready())


class TestShippedModelPath(unittest.TestCase):
  """A file counts only when it is the model we mean, at the size we expect.
  Models live one file per oid so switching back does not re-download."""

  MODEL = {'name': 'Alpha', 'ref': REF_A, 'oid': 'a' * 64, 'size': 4096}

  def setUp(self):
    self.root = tempfile.mkdtemp()
    paths = mock.patch('openpilot.sunnypilot.accelerators.jetlink.helpers.Paths')
    self.addCleanup(paths.stop)
    paths.start().model_root.return_value = self.root
    chosen = mock.patch.object(helpers, 'selected_model', return_value=self.MODEL)
    self.addCleanup(chosen.stop)
    chosen.start()

  def fetched(self, size: int) -> Path:
    path = helpers.model_dir() / helpers.model_file_name(self.MODEL)
    path.parent.mkdir()
    path.write_bytes(b'\0' * size)
    return path

  def test_a_model_not_resolved_yet_is_no_path(self):
    with mock.patch.object(helpers, 'selected_model', return_value={**self.MODEL, 'oid': None, 'size': None}):
      assert helpers.shipped_model_path() is None

  def test_nothing_fetched_yet(self):
    assert helpers.shipped_model_path() is None

  def test_the_chosen_model_is_accepted(self):
    fetched = self.fetched(4096)
    assert helpers.shipped_model_path() == fetched

  def test_a_truncated_download_is_rejected(self):
    # Half a model is exactly what must not reach TensorRT.
    self.fetched(2048)
    assert helpers.shipped_model_path() is None

  def test_no_model_chosen_is_no_path(self):
    with mock.patch.object(helpers, 'selected_model', return_value=None):
      assert helpers.shipped_model_path() is None
