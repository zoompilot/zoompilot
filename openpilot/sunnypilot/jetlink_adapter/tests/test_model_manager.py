"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

sunnypilot's model manager with jetlink and no chestnut: the big-model slot
is jetlink's pick, and the big-model catalog carries the models newer
catalogs list, so a big model sunnypilot publishes after this build is still
pickable with a Jetson. Against the real manager, the real adapter and the
pinned jetlink, with the network mocked.
"""
from __future__ import annotations

import copy
import unittest
from unittest import mock

import requests

from jetlink.openpilot.settings import FileParams, Settings
from jetlink.spec import ModelSpec

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.jetlink_adapter import KEYS, MODES
from openpilot.sunnypilot.models import helpers as model_helpers, manager as manager_module
from openpilot.sunnypilot.models.fetcher import ModelFetcher, ModelParser
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION, _bundle_needs_reset, resolve_bundle_by_ref

OLD, NEW = 'a' * 40, 'e' * 40
V3, V4 = '3' * 40, '4' * 40
V4_OID = 'c' * 64


def bundle(ref: str, index: int, name: str, selector=REQUIRED_JSON_VERSION, big: bool = True) -> dict:
  return {'ref': ref, 'index': index, 'minimum_selector_version': str(selector), 'is_big': big, 'is_20hz': True,
          'display_name': name, 'short_name': name[:4], 'generation': '12', 'environment': 'development',
          'runner': 'tinygrad', 'build_time': '2026-09-25T00:00:00Z', 'overrides': {'folder': 'Master Models'},
          'models': [{'type': 'chunked', 'artifact': {'file_name': f'{name}.pkl', 'download_uri': {'url': 'x', 'sha256': 'y'}}}]}


PINNED = {'tinygrad_ref': 'pinned', 'bundles': [bundle(OLD, 12, 'Cinque Terre V3')]}
NEWER = {'tinygrad_ref': 'next', 'bundles': [bundle(OLD, 12, 'Cinque Terre V3', REQUIRED_JSON_VERSION + 1),
                                             bundle(NEW, 13, 'Cinque Terre V4', REQUIRED_JSON_VERSION + 1)]}


def chestnut(fitted: bool):
  """comma's chestnut, fitted or not, as the adapter and the model manager each ask."""
  return (mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=fitted),
          mock.patch.object(model_helpers, 'chestnut_present', return_value=fitted))


class JetlinkTest(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    # a binding of its own, so no cache of another test's params answers here
    p = mock.patch.object(jetlink_adapter, '_bound', None)
    p.start()
    self.addCleanup(p.stop)


class TestBigCatalog(JetlinkTest):
  def merged(self, newer=NEWER):
    probe_result = {'side_effect': newer} if isinstance(newer, Exception) else {'return_value': newer}
    with mock.patch('jetlink.registry.catalog.fetch_catalogs', **probe_result) as probe:
      out = jetlink_adapter.extend_catalog(PINNED)
    return out, probe

  def test_a_model_only_a_newer_catalog_lists_can_be_picked(self):
    out, probe = self.merged()
    probe.assert_called_once_with()
    bundles = ModelParser.parse_models(out)
    self.assertEqual([b.ref for b in bundles], [OLD, NEW])
    picked, source = resolve_bundle_by_ref(NEW, {'chestnut': bundles})
    self.assertEqual((picked.displayName, source), ('Cinque Terre V4', 'chestnut'))
    # nothing for a chestnut to fetch
    self.assertEqual(list(picked.models), [])
    # the slot written from it survives the model manager's validation
    self.assertFalse(_bundle_needs_reset(picked, bundles, check_files=False))

  def test_a_model_the_pinned_catalog_has_keeps_its_build(self):
    out, _ = self.merged()
    self.assertIs(out['bundles'][0], PINNED['bundles'][0])
    self.assertEqual(out['tinygrad_ref'], 'pinned')

  def test_nothing_newer_or_a_failed_probe_leaves_it_alone(self):
    self.assertIs(self.merged(newer=PINNED)[0], PINNED)
    self.assertIs(self.merged(newer=OSError('offline'))[0], PINNED)

  def test_a_failed_probe_keeps_what_the_last_one_found(self):
    # the model manager's cached copy has the newer model as the merge made it; a
    # probe that fails now must not drop a pick that is only listed there
    last, _ = self.merged()
    Params().put(KEYS.catalog, {**last, ModelFetcher.EXTENDED_KEY: True}, block=True)
    out, _ = self.merged(newer=OSError('offline'))
    self.assertEqual([b['ref'] for b in out['bundles']], [OLD, NEW])
    # sunnypilot's own entries come from the fetch, never the cache
    self.assertIs(out['bundles'][0], PINNED['bundles'][0])
    self.assertEqual(ModelParser.parse_models(out)[1].ref, NEW)


class TestExtendsCatalog(JetlinkTest):
  """Hardware, not the link setting: the model manager drops a pick its catalog does
  not list, so a catalog that followed the setting lost one on a boot with it off."""

  def extends(self, fitted: bool, link: str) -> bool:
    Params().put(KEYS.link, MODES.index(link), block=True)
    a, b = chestnut(fitted)
    with a, b, mock.patch.object(jetlink_adapter, '_bound', None):
      return jetlink_adapter.should_extend_catalog()

  def test_without_a_chestnut_it_is_extended_whatever_the_setting(self):
    for link in MODES:
      self.assertTrue(self.extends(False, link), link)

  def test_a_chestnut_leaves_it_as_fetched(self):
    for link in MODES:
      self.assertFalse(self.extends(True, link), link)


class TestFetcherHook(OpenpilotTestCase):
  """The model manager asks once per fetch, for the big-model source only, and the
  cache records the answer."""

  def fetch(self, source, extends=True):
    response = mock.MagicMock(status_code=200)
    response.json.return_value = dict(PINNED)
    params = mock.MagicMock()
    with mock.patch('openpilot.sunnypilot.models.fetcher.requests.get', return_value=response), \
         mock.patch.object(jetlink_adapter, 'should_extend_catalog', return_value=extends) as asked, \
         mock.patch.object(jetlink_adapter, 'extend_catalog', side_effect=lambda c: c) as hook:
      ModelFetcher(params)._fetch_and_cache_models(source)
    cached = next((c.args[1] for c in params.put.call_args_list if c.args[0] == KEYS.catalog), None)
    return hook, asked, cached

  def test_the_big_model_source_is_extended_and_says_so(self):
    hook, asked, cached = self.fetch('chestnut')
    hook.assert_called_once_with(PINNED)
    asked.assert_called_once_with()
    self.assertIs(cached[ModelFetcher.EXTENDED_KEY], True)

  def test_beside_a_chestnut_it_is_cached_as_fetched(self):
    hook, _, cached = self.fetch('chestnut', extends=False)
    hook.assert_not_called()
    self.assertEqual(cached, {**PINNED, ModelFetcher.EXTENDED_KEY: False})

  def test_the_small_model_source_is_not(self):
    hook, asked, _ = self.fetch('qcom')
    hook.assert_not_called()
    asked.assert_not_called()


class TestCatalogFollowsTheHardware(OpenpilotTestCase):
  """A catalog cached beside a chestnut hides the newer models for an hour once it
  comes out, and the reverse. Refetch when the hardware changes."""

  def setUp(self):
    super().setUp()
    self.fetcher = ModelFetcher(mock.MagicMock())
    self.refetch = mock.patch.object(self.fetcher, '_fetch_and_cache_models', return_value=[]).start()
    self.addCleanup(mock.patch.stopall)

  def bundles(self, stamped, extends):
    cached = {**PINNED, ModelFetcher.EXTENDED_KEY: stamped}
    with mock.patch.object(self.fetcher.model_caches['chestnut'], 'get', return_value=(cached, False)), \
         mock.patch.object(jetlink_adapter, 'should_extend_catalog', return_value=extends):
      self.fetcher.get_bundles_for_source('chestnut')

  def test_a_cache_from_the_same_hardware_is_used(self):
    self.bundles(stamped=True, extends=True)
    self.bundles(stamped=False, extends=False)
    self.refetch.assert_not_called()

  def test_a_chestnut_coming_out_refetches(self):
    self.bundles(stamped=False, extends=True)
    self.refetch.assert_called_once_with('chestnut')

  def test_offline_it_is_tried_once_per_change(self):
    self.refetch.return_value = None   # the fetch failed; the old cache stands
    for _ in range(3):
      self.bundles(stamped=False, extends=True)
    self.assertEqual(self.refetch.call_count, 1)
    self.bundles(stamped=True, extends=False)   # a chestnut back in
    self.assertEqual(self.refetch.call_count, 2)

  def test_the_small_model_source_never_asks(self):
    with mock.patch.object(self.fetcher.model_caches['qcom'], 'get', return_value=({'bundles': []}, False)), \
         mock.patch.object(jetlink_adapter, 'should_extend_catalog') as extends:
      self.fetcher.get_bundles_for_source('qcom')
    extends.assert_not_called()


# -- sunnypilot's "refresh model list" ----------------------------------------
# The panels zero the model manager's two sync keys and spin until both are
# stamped again (selfdrive/ui/sunnypilot/model_info.py). The manager's next tick
# refetches both catalogs; the big-model one is extended with the newer
# catalogs, and its slot is jetlink's pick, which the owner watches along with
# the built model's record by mtime. Each test runs the manager's real loop for
# one tick against its own params.

SMALL = {'bundles': [bundle('1' * 40, 30, 'North Dakota', big=False)]}
BIG = {'bundles': [bundle(V3, 12, 'Cinque Terre V3')]}
LATER = {'bundles': [bundle(V4, 13, 'Cinque Terre V4', selector=REQUIRED_JSON_VERSION + 1)]}


class _Tick(BaseException):
  """Ends main_thread after one pass. An Exception would be the loop's to swallow."""


class RefreshTest(JetlinkTest):
  CHESTNUT = False

  def setUp(self):
    super().setUp()
    self.params = Params()
    self.offline = False
    self.newer: dict | Exception = LATER
    # the checkout carries the warp for this camera, so readiness is the pick's engine alone
    for patcher in (
      mock.patch('openpilot.sunnypilot.models.fetcher.requests.get', side_effect=self.serve),
      mock.patch('jetlink.registry.catalog.fetch_catalogs', side_effect=self.probe),
      *chestnut(self.CHESTNUT),
    ):
      patcher.start()
      self.addCleanup(patcher.stop)
    self.new_process()
    self.addCleanup(model_helpers._LAST_VALIDATED_RAW.clear)

  def serve(self, url, timeout=None):
    if self.offline:
      raise requests.exceptions.ConnectionError("offline")
    response = mock.MagicMock(status_code=200)
    response.json.return_value = copy.deepcopy({ModelFetcher.MODEL_URL: SMALL, ModelFetcher.MODEL_URL_CHESTNUT: BIG}[url])
    return response

  def probe(self):
    if isinstance(self.newer, Exception):
      raise self.newer
    return copy.deepcopy(self.newer)

  def new_process(self) -> None:
    """The manager runs offroad only, so each parked period is a fresh process."""
    model_helpers._LAST_VALIDATED_RAW.clear()
    jetlink_adapter._bound = None
    with mock.patch.object(manager_module.messaging, 'PubMaster'), mock.patch.object(manager_module.messaging, 'SubMaster'):
      self.manager = manager_module.ModelManagerSP()
    self.manager.sm.__getitem__.return_value.chestnutPresent = self.CHESTNUT

  def tick(self) -> None:
    with mock.patch.object(manager_module, 'Ratekeeper') as rk, \
         mock.patch.object(manager_module.cloudlog, 'exception', wraps=manager_module.cloudlog.exception) as logged:
      rk.return_value.keep_time.side_effect = _Tick
      with self.assertRaises(_Tick):
        self.manager.main_thread()
    # the loop swallows what the tick raised, and says so
    self.assertFalse([c for c in logged.call_args_list if str(c.args[0]).startswith("Error in main thread")])
    # what jetlink keeps of the catalog and the pick lasts two seconds
    jetlink_adapter._bound = None

  def refresh(self) -> None:
    """What the panels' button does (model_info.refresh_model_list), then the manager's next tick."""
    for cache in self.manager.model_fetcher.model_caches.values():
      self.params.put(cache._LAST_SYNC_KEY, 0, block=True)
    self.tick()

  def stamps(self) -> list:
    return [self.params.get(cache._LAST_SYNC_KEY) for cache in self.manager.model_fetcher.model_caches.values()]

  def pick(self, ref: str) -> None:
    """Pick a big model as the manager stores it without a chestnut: the catalog's
    entry, files not fetched (ModelManagerSP._download_bundle)."""
    bundles = ModelParser.parse_models(self.params.get(KEYS.catalog))
    self.params.put(KEYS.big_model, resolve_bundle_by_ref(ref, {"chestnut": bundles})[0].to_dict(), block=True)

  def marks(self) -> dict:
    """What the owner stats to decide a provisioning run is due."""
    return Settings(FileParams(jetlink_adapter.owner_config().params_dir), jetlink_adapter.Adapter().keys).marks()


class TestRefreshOnAJetlinkDevice(RefreshTest):
  def setUp(self):
    super().setUp()
    self.params.put(KEYS.link, MODES.index('usb'), block=True)
    self.assertTrue(jetlink_adapter.should_run(False, self.params, None))
    self.tick()   # the catalogs as the manager had them before anyone pressed refresh
    self.pick(V4)
    # the pick's pointer is resolved and the Jetson has built its engine
    self.params.put(KEYS.pointers, {V4: {'oid': V4_OID, 'size': 766_000_000}}, block=True)
    spec = ModelSpec(sha256=V4_OID, nbytes=766_000_000, frame_skip=4, input_shapes={'features_buffer': (1, 24, 512)},
                     output_shapes={'outputs': (1, 16)}, output_slices={'plan': slice(0, 16)}, checkpoint=None)
    self.params.put(KEYS.spec, {**spec.to_dict(), 'ready': True}, block=True)
    self.tick()
    self.slot, self.spec, self.before = self.params.get(KEYS.big_model), self.params.get(KEYS.spec), self.marks()
    self.assert_pick_and_readiness_kept()

  def assert_pick_and_readiness_kept(self):
    self.assertEqual(self.params.get(KEYS.big_model), self.slot)
    self.assertEqual(self.params.get(KEYS.spec), self.spec)
    status = jetlink_adapter.status()
    self.assertEqual(status.model, 'Cinque Terre V4')
    self.assertEqual(status.ready, status.enabled, "the pick's engine no longer counts as built")
    self.assertEqual(self.marks(), self.before, "the owner would start a provisioning run")

  def test_a_refresh_stamps_both_catalogs_so_the_spinner_ends(self):
    self.refresh()
    self.assertTrue(all(self.stamps()), self.stamps())
    cached = self.params.get(KEYS.catalog)
    self.assertIs(cached[ModelFetcher.EXTENDED_KEY], True)
    self.assertEqual([b['ref'] for b in cached['bundles']], [V3, V4])
    self.assert_pick_and_readiness_kept()

  def test_a_failed_fetch_leaves_the_pick_and_readiness_alone(self):
    self.offline = True
    self.refresh()
    # nothing restamped: the panels' spinner gives up at its timeout, as upstream
    self.assertEqual(self.stamps(), [0, 0])
    self.assertEqual(self.params.get(KEYS.catalog)['bundles'][1]['ref'], V4)
    self.assert_pick_and_readiness_kept()

  def test_the_pick_survives_a_catalog_that_no_longer_lists_it(self):
    # the pick is the slot and its pointer, not a catalog row
    self.newer = {'bundles': []}
    self.refresh()
    self.assertTrue(all(self.stamps()))
    self.assertNotIn(V4, [b['ref'] for b in self.params.get(KEYS.catalog)['bundles']])
    self.assert_pick_and_readiness_kept()

  def test_a_failed_probe_for_the_newer_catalogs_keeps_the_pick_listed(self):
    # sunnypilot's catalog came, jetlink's did not: the models the last probe found
    # stay listed, or the manager's next start would drop the pick and the owner
    # would build the default model's engine in its place
    self.newer = OSError("offline")
    self.refresh()
    self.assertTrue(all(self.stamps()))
    self.assertIn(V4, [b['ref'] for b in self.params.get(KEYS.catalog)['bundles']])
    self.new_process()
    self.tick()
    self.assert_pick_and_readiness_kept()

  def test_with_the_link_off_a_refresh_is_the_same(self):
    # the catalog follows the hardware, not the setting, so a pick made with the
    # link off is still listed when it is turned on
    self.params.put(KEYS.link, MODES.index('off'), block=True)
    self.assertFalse(jetlink_adapter.should_run(False, self.params, None))
    self.before = self.marks()
    self.refresh()
    self.assertTrue(all(self.stamps()))
    self.assertIs(self.params.get(KEYS.catalog)[ModelFetcher.EXTENDED_KEY], True)
    self.assert_pick_and_readiness_kept()


class TestRefreshBesideAChestnut(RefreshTest):
  """A chestnut runs sunnypilot's catalog as fetched; jetlink adds nothing to the refresh."""
  CHESTNUT = True

  def test_the_catalog_is_the_one_fetched(self):
    self.tick()
    self.refresh()
    self.assertTrue(all(self.stamps()))
    self.assertEqual(self.params.get(KEYS.catalog), {**BIG, ModelFetcher.EXTENDED_KEY: False})


if __name__ == '__main__':
  unittest.main()
