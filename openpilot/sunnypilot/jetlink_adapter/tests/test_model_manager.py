"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

sunnypilot's model manager with jetlink and no chestnut: the big-model slot
is jetlink's pick, and the big-model list is jetlink's model index, every big
model a Jetson can run, in its folders. Against the real manager, the real
adapter and the pinned jetlink, with the network mocked.
"""
from __future__ import annotations

import copy
import unittest
from unittest import mock

import requests

from jetlink.openpilot.settings import FileParams, Settings
from jetlink.registry.index import NetworkError, parse_index
from jetlink.spec import ModelSpec

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.jetlink_adapter import KEYS, MODES
from openpilot.sunnypilot.models import helpers as model_helpers, manager as manager_module
from openpilot.sunnypilot.models.fetcher import ModelCache, ModelParser
from openpilot.sunnypilot.models.fetcher_zp import JETLINK_KEY, ModelFetcherZP
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION, _bundle_needs_reset, resolve_bundle_by_ref

V3, V4, V41 = '3' * 40, '4' * 40, '5' * 40
V3_OID, V4_OID = 'b' * 64, 'c' * 64


def bundle(ref: str, index: int, name: str, selector=REQUIRED_JSON_VERSION, big: bool = True) -> dict:
  """A bundle as sunnypilot's catalogs carry it."""
  return {'ref': ref, 'index': index, 'minimum_selector_version': str(selector), 'is_big': big, 'is_20hz': True,
          'display_name': name, 'short_name': name[:4], 'generation': '12', 'environment': 'development',
          'runner': 'tinygrad', 'build_time': '2026-09-25T00:00:00Z', 'overrides': {'folder': 'Master Models'},
          'models': [{'type': 'chunked', 'artifact': {'file_name': f'{name}.pkl', 'download_uri': {'url': 'x', 'sha256': 'y'}}}]}


def entry(ref: str, name: str, oid: str, folder: str = 'Master Models', short_name: str | None = None) -> dict:
  """A listed model as the model index carries it."""
  return {'ref': ref, 'name': name, 'oid': oid, 'size': 766_000_000, 'folder': folder, 'short_name': short_name or name[:4]}


def index_json(*listed, refs=None) -> dict:
  return {'schema': 1, 'models': list(listed), 'refs': refs or {}, 'onnx': {}}


INDEX = index_json(entry(V4, 'Cinque Terre V4 (October 01, 2026)', V4_OID, '2026 Deep RL Models'), entry(V3, 'Cinque Terre V3', V3_OID))
PINNED = {'tinygrad_ref': 'pinned', 'bundles': [bundle(V3, 12, 'Cinque Terre V3')]}
# jetlink's list, as big_catalog() makes it of INDEX
INDEX_BUNDLES = [{**bundle(V4, 2, 'Cinque Terre V4'), 'models': []}, {**bundle(V3, 1, 'Cinque Terre V3'), 'models': []}]


def serving(data):
  """jetlink's fetch of the model index, answered with `data` (an Exception: it fails)."""
  def fetch(*_, **__):
    current = data() if callable(data) else data
    if isinstance(current, Exception):
      raise current
    return copy.deepcopy(current), parse_index(current)
  return mock.patch('jetlink.registry.index.fetch_index', side_effect=fetch)


def chestnut(fitted: bool):
  """comma's chestnut, fitted or not, as the adapter and the model manager each ask."""
  return (mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=fitted),
          mock.patch.object(model_helpers, 'chestnut_present', return_value=fitted))


class JetlinkTest(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    # a binding of its own, so no cache of another test's params answers here
    p = mock.patch.object(jetlink_adapter, '_hooks', None)
    p.start()
    self.addCleanup(p.stop)


class TestBigCatalog(JetlinkTest):
  def test_the_index_is_a_list_the_model_manager_takes(self):
    with serving(INDEX):
      out = jetlink_adapter.big_catalog()
    bundles = ModelParser.parse_models(out)
    self.assertEqual([(b.ref, {o.key: o.value for o in b.overrides}['folder']) for b in bundles],
                     [(V4, '2026 Deep RL Models'), (V3, 'Master Models')])
    picked, source = resolve_bundle_by_ref(V4, {'chestnut': bundles})
    self.assertEqual((picked.displayName, source), ('Cinque Terre V4 (October 01, 2026)', 'chestnut'))
    # nothing for a chestnut to fetch
    self.assertEqual(list(picked.models), [])
    # the slot written from it survives the model manager's validation
    self.assertFalse(_bundle_needs_reset(picked, bundles, check_files=False))

  def test_offline_there_is_no_list_and_the_model_manager_keeps_its_own(self):
    with serving(NetworkError('offline')):
      self.assertIsNone(jetlink_adapter.big_catalog())

  def test_a_recompile_is_found_under_the_ref_that_lists_its_onnx(self):
    with serving(index_json(entry(V41, 'Cinque Terre V4.1', V4_OID), refs={V4: {'oid': V4_OID, 'size': 766_000_000}})):
      jetlink_adapter.big_catalog()
    self.assertEqual(jetlink_adapter.listed_ref(V4), V41)
    self.assertIsNone(jetlink_adapter.listed_ref(V3))


class TestListsBigModels(JetlinkTest):
  """Hardware, not the link setting: the model manager drops a pick its catalog does
  not list, so a list that followed the setting lost one on a boot with it off."""

  def lists(self, fitted: bool, link: str) -> bool:
    Params().put(KEYS.link, MODES.index(link), block=True)
    a, b = chestnut(fitted)
    with a, b, mock.patch.object(jetlink_adapter, '_hooks', None):
      return jetlink_adapter.lists_big_models()

  def test_without_a_chestnut_it_is_jetlinks_whatever_the_setting(self):
    for link in MODES:
      self.assertTrue(self.lists(False, link), link)

  def test_with_a_chestnut_it_is_sunnypilots(self):
    for link in MODES:
      self.assertFalse(self.lists(True, link), link)


class TestFetcherHook(OpenpilotTestCase):
  """The model manager asks once per fetch, for the big-model source only, and the
  cache records whose list it is."""

  def fetch(self, source, jetlink=True):
    response = mock.MagicMock(status_code=200)
    response.json.return_value = dict(PINNED)
    params = mock.MagicMock()
    params.get.return_value = None
    with mock.patch('openpilot.sunnypilot.models.fetcher.requests.get', return_value=response) as get, \
         mock.patch.object(jetlink_adapter, 'lists_big_models', return_value=jetlink) as asked, \
         mock.patch.object(jetlink_adapter, 'big_catalog', return_value={'bundles': INDEX_BUNDLES}) as hook:
      parsed = ModelFetcherZP(params)._fetch_and_cache_models(source)
    cached = next((c.args[1] for c in params.put.call_args_list if c.args[0] == KEYS.catalog), None)
    return hook, asked, get, cached, parsed

  def test_without_a_chestnut_the_big_model_list_is_jetlinks_and_says_so(self):
    hook, asked, get, cached, _ = self.fetch('chestnut')
    hook.assert_called_once_with()
    asked.assert_called_once_with()
    get.assert_not_called()
    self.assertIs(cached[JETLINK_KEY], True)
    self.assertEqual([b['ref'] for b in cached['bundles']], [V4, V3])

  def test_offline_nothing_is_cached_and_the_old_list_stands(self):
    params = mock.MagicMock(**{'get.return_value': None})
    with mock.patch('openpilot.sunnypilot.models.fetcher.requests.get') as get, \
         mock.patch.object(jetlink_adapter, 'lists_big_models', return_value=True), \
         mock.patch.object(jetlink_adapter, 'big_catalog', return_value=None):
      self.assertIsNone(ModelFetcherZP(params)._fetch_and_cache_models('chestnut'))
    get.assert_not_called()
    self.assertNotIn(KEYS.catalog, [c.args[0] for c in params.put.call_args_list])

  def test_beside_a_chestnut_it_is_sunnypilots_as_fetched(self):
    hook, _, get, cached, _ = self.fetch('chestnut', jetlink=False)
    hook.assert_not_called()
    get.assert_called_once()
    self.assertEqual(cached, PINNED)

  def test_the_small_model_source_never_asks(self):
    hook, asked, _, _, _ = self.fetch('qcom')
    hook.assert_not_called()
    asked.assert_not_called()


class TestCatalogFollowsTheHardware(OpenpilotTestCase):
  """A list cached beside a chestnut is sunnypilot's for an hour once it comes out,
  and the reverse. Refetch when the hardware changes."""

  def setUp(self):
    super().setUp()
    self.fetcher = ModelFetcherZP(mock.MagicMock(**{'get.return_value': None}))
    self.refetch = mock.patch.object(self.fetcher, '_fetch_and_cache_models', return_value=[]).start()
    self.addCleanup(mock.patch.stopall)

  def bundles(self, stamped, jetlink):
    cached = {**PINNED, **({JETLINK_KEY: True} if stamped else {})}
    with mock.patch.object(ModelCache, 'get', return_value=(cached, False)), \
         mock.patch.object(jetlink_adapter, 'lists_big_models', return_value=jetlink):
      self.fetcher.get_bundles_for_source('chestnut')

  def test_a_cache_from_the_same_hardware_is_used(self):
    self.bundles(stamped=True, jetlink=True)
    self.bundles(stamped=False, jetlink=False)
    self.refetch.assert_not_called()

  def test_a_chestnut_coming_out_refetches(self):
    self.bundles(stamped=False, jetlink=True)
    self.refetch.assert_called_once_with('chestnut')

  def test_offline_it_is_tried_once_per_change(self):
    self.refetch.return_value = None   # the fetch failed; the old cache stands
    for _ in range(3):
      self.bundles(stamped=False, jetlink=True)
    self.assertEqual(self.refetch.call_count, 1)
    self.bundles(stamped=True, jetlink=False)   # a chestnut back in
    self.assertEqual(self.refetch.call_count, 2)

  def test_the_small_model_source_never_asks(self):
    with mock.patch.object(ModelCache, 'get', return_value=({'bundles': []}, False)), \
         mock.patch.object(jetlink_adapter, 'lists_big_models') as lists:
      self.fetcher.get_bundles_for_source('qcom')
    lists.assert_not_called()


# -- sunnypilot's "refresh model list" ----------------------------------------
# The panels zero the model manager's two sync keys and spin until both are
# stamped again (selfdrive/ui/sunnypilot/model_info.py). The manager's next tick
# refetches both catalogs; the big-model one is jetlink's list, the model
# index, and its slot is jetlink's pick, which the owner watches along with the
# built model's record by mtime. Each test runs the manager's real loop for one
# tick against its own params.

SMALL = {'bundles': [bundle('1' * 40, 30, 'North Dakota', big=False)]}
BIG = {'bundles': [bundle(V3, 12, 'Cinque Terre V3')]}


class _Tick(BaseException):
  """Ends main_thread after one pass. An Exception would be the loop's to swallow."""


class RefreshTest(JetlinkTest):
  CHESTNUT = False

  def setUp(self):
    super().setUp()
    self.params = Params()
    self.offline = False
    self.index: dict | Exception = INDEX
    # the checkout carries the warp for this camera, so readiness is the pick's engine alone
    for patcher in (
      mock.patch('openpilot.sunnypilot.models.fetcher.requests.get', side_effect=self.serve),
      serving(lambda: NetworkError('offline') if self.offline else self.index),
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
    response.json.return_value = copy.deepcopy({ModelFetcherZP.MODEL_URL: SMALL, ModelFetcherZP.MODEL_URL_CHESTNUT: BIG}[url])
    return response

  def new_process(self) -> None:
    """The manager runs offroad only, so each parked period is a fresh process."""
    model_helpers._LAST_VALIDATED_RAW.clear()
    jetlink_adapter._hooks = None
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
    # what jetlink keeps of the index and the pick lasts two seconds
    jetlink_adapter._hooks = None

  def refresh(self) -> None:
    """What the panels' button does (model_info.refresh_model_list), then the manager's next tick."""
    for cache in self.manager.model_fetcher.model_caches.values():
      self.params.put(cache._LAST_SYNC_KEY, 0, block=True)
    self.tick()

  def stamps(self) -> list:
    return [self.params.get(cache._LAST_SYNC_KEY) for cache in self.manager.model_fetcher.model_caches.values()]

  def listed(self) -> list[str]:
    return [b['ref'] for b in self.params.get(KEYS.catalog)['bundles']]

  def pick(self, ref: str) -> None:
    """Pick a big model as the manager stores it without a chestnut: the catalog's
    entry, files not fetched (ModelManagerSP._download_bundle)."""
    bundles = ModelParser.parse_models(self.params.get(KEYS.catalog))
    self.params.put(KEYS.big_model, resolve_bundle_by_ref(ref, {"chestnut": bundles})[0].to_dict(), block=True)

  def marks(self) -> dict:
    """What the owner stats to decide a provisioning run is due."""
    config = jetlink_adapter.owner_config()
    return Settings(FileParams(config.params_dir), config.keys).marks()


class TestRefreshOnAJetlinkDevice(RefreshTest):
  def setUp(self):
    super().setUp()
    self.params.put(KEYS.link, MODES.index('usb'), block=True)
    self.assertTrue(jetlink_adapter.should_run(False, self.params, None))
    self.tick()   # the catalogs as the manager had them before anyone pressed refresh
    self.pick(V4)
    # the Jetson has built the pick's engine: the index names its ONNX
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
    self.assertEqual(status.model, 'Cinque Terre V4 (October 01, 2026)')
    self.assertEqual(status.ready, status.enabled, "the pick's engine no longer counts as built")
    self.assertEqual(self.marks(), self.before, "the owner would start a provisioning run")

  def test_a_refresh_stamps_both_catalogs_so_the_spinner_ends(self):
    self.refresh()
    self.assertTrue(all(self.stamps()), self.stamps())
    self.assertIs(self.params.get(KEYS.catalog)[JETLINK_KEY], True)
    self.assertEqual(self.listed(), [V4, V3])
    self.assertEqual(self.params.get("ModelManager_ActiveJson"), {"qcom": ModelFetcherZP.MODEL_URL})
    self.assert_pick_and_readiness_kept()

  def test_offline_the_pick_and_readiness_stand(self):
    self.offline = True
    self.refresh()
    # nothing restamped: the panels' spinner gives up at its timeout, as upstream
    self.assertEqual(self.stamps(), [0, 0])
    self.assertEqual(self.listed(), [V4, V3])
    self.new_process()
    self.tick()
    self.assert_pick_and_readiness_kept()

  def test_an_index_that_cannot_be_had_keeps_the_list(self):
    # sunnypilot's small-model catalog came, the index did not: the big-model
    # list stands, or the manager's next start would drop the pick and the
    # owner would build the default model's engine in its place
    self.index = NetworkError('offline')
    self.refresh()
    self.assertEqual(self.listed(), [V4, V3])
    self.new_process()
    self.tick()
    self.assert_pick_and_readiness_kept()

  def test_a_pick_the_index_lists_under_a_newer_ref_follows_it(self):
    # a recompile of the same ONNX is listed once, under its newest ref and a
    # short name of its own: the pick carries over to it by its ONNX, and the
    # engine already built still counts
    self.index = index_json(entry(V41, 'Cinque Terre V4.1', V4_OID, short_name='CTV41'), entry(V3, 'Cinque Terre V3', V3_OID),
                            refs={V4: {'oid': V4_OID, 'size': 766_000_000}})
    self.refresh()
    self.assertEqual(self.listed(), [V41, V3])
    # the manager holds the slot to the list at its next start (validate_active_bundles)
    self.new_process()
    for _ in range(3):
      self.tick()
    self.assertEqual(self.params.get(KEYS.big_model)['ref'], V41)
    status = jetlink_adapter.status()
    self.assertEqual(status.model, 'Cinque Terre V4.1')
    self.assertEqual(status.ready, status.enabled)

  def test_with_the_link_off_a_refresh_is_the_same(self):
    # the list follows the hardware, not the setting, so a pick made with the
    # link off is still listed when it is turned on
    self.params.put(KEYS.link, MODES.index('off'), block=True)
    self.assertFalse(jetlink_adapter.should_run(False, self.params, None))
    self.before = self.marks()
    self.refresh()
    self.assertTrue(all(self.stamps()))
    self.assertIs(self.params.get(KEYS.catalog)[JETLINK_KEY], True)
    self.assert_pick_and_readiness_kept()


class TestRefreshBesideAChestnut(RefreshTest):
  """A chestnut runs sunnypilot's catalog as fetched; jetlink adds nothing to the refresh."""
  CHESTNUT = True

  def test_the_catalog_is_the_one_fetched(self):
    self.tick()
    self.refresh()
    self.assertTrue(all(self.stamps()))
    self.assertEqual(self.params.get(KEYS.catalog), BIG)
    self.assertEqual(self.params.get("ModelManager_ActiveJson"), {
      "qcom": ModelFetcherZP.MODEL_URL,
      "chestnut": ModelFetcherZP.MODEL_URL_CHESTNUT,
    })


class TestActiveJson(OpenpilotTestCase):
  """Sunnylink UI queries ModelManager_ActiveJson to decide whether to fetch
  model manifests directly from GitHub or fall back to device cache. While the
  big-model list is jetlink's, the chestnut URL is omitted so Sunnylink UI pulls
  it from ModelManager_ModelsCache_Chestnut on the device."""

  def test_active_json_omits_chestnut_when_the_cached_list_is_jetlinks(self):
    params = Params()
    params.put(KEYS.catalog, {"bundles": [], JETLINK_KEY: True}, block=True)
    ModelFetcherZP(params)
    self.assertEqual(params.get("ModelManager_ActiveJson"), {"qcom": ModelFetcherZP.MODEL_URL})

  def test_active_json_includes_chestnut_when_the_cached_list_is_sunnypilots(self):
    params = Params()
    params.put(KEYS.catalog, {"bundles": []}, block=True)
    ModelFetcherZP(params)
    self.assertEqual(params.get("ModelManager_ActiveJson"), {
      "qcom": ModelFetcherZP.MODEL_URL,
      "chestnut": ModelFetcherZP.MODEL_URL_CHESTNUT,
    })


if __name__ == '__main__':
  unittest.main()
