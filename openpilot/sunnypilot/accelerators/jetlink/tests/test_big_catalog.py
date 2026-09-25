"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A big model sunnypilot publishes after this build is still pickable with a
Jetson: the model manager's big-model catalog has the newer catalogs' models
folded in, as entries the Jetson can run and a chestnut never downloads.
"""
from __future__ import annotations

import unittest
from unittest import mock

from openpilot.sunnypilot.accelerators.jetlink import backend
from openpilot.sunnypilot.models.fetcher import ModelFetcher, ModelParser
from openpilot.sunnypilot.models.helpers import _bundle_needs_reset, resolve_bundle_by_ref

OLD, NEW = 'a' * 40, 'e' * 40
URL = ModelFetcher.MODEL_URL_CHESTNUT


def bundle(ref: str, index: int, selector: str, name: str) -> dict:
  return {'ref': ref, 'index': index, 'minimum_selector_version': selector, 'is_big': True, 'is_20hz': True,
          'display_name': name, 'short_name': name[:4], 'generation': '12', 'environment': 'development',
          'runner': 'tinygrad', 'build_time': '2026-09-25T00:00:00Z', 'overrides': {'folder': 'Master Models'},
          'models': [{'type': 'chunked', 'artifact': {'file_name': f'{name}.pkl', 'download_uri': {'url': 'x', 'sha256': 'y'}}}]}


PINNED = {'tinygrad_ref': 'pinned', 'bundles': [bundle(OLD, 12, '19', 'Cinque Terre V3')]}
NEWER = [{'tinygrad_ref': 'next', 'bundles': [bundle(OLD, 12, '20', 'Cinque Terre V3'), bundle(NEW, 13, '20', 'Cinque Terre V4')]}]


class TestBigCatalog(unittest.TestCase):
  def merged(self, enabled=True, chestnut=False, newer=NEWER):
    with mock.patch.object(backend.helpers, 'enabled', return_value=enabled), \
         mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=chestnut), \
         mock.patch('jetlink.registry.catalog.newer_catalogs', return_value=newer) as probe:
      out = backend.big_catalog(PINNED, URL)
    return out, probe

  def test_a_model_only_a_newer_catalog_lists_can_be_picked(self):
    out, probe = self.merged()
    probe.assert_called_once_with(URL)
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

  def test_the_link_off_or_a_chestnut_leaves_the_catalog_alone(self):
    for kwargs in ({'enabled': False}, {'chestnut': True}):
      with self.subTest(**kwargs):
        out, probe = self.merged(**kwargs)
        self.assertIs(out, PINNED)
        probe.assert_not_called()

  def test_nothing_newer_or_a_failed_probe_leaves_it_alone(self):
    self.assertIs(self.merged(newer=[])[0], PINNED)
    with mock.patch.object(backend.helpers, 'enabled', return_value=True), \
         mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=False), \
         mock.patch('jetlink.registry.catalog.newer_catalogs', side_effect=OSError('offline')):
      self.assertIs(backend.big_catalog(PINNED, URL), PINNED)


class TestFetcherHook(unittest.TestCase):
  """The model manager asks once per fetch, for the big-model source only."""

  def fetch(self, source):
    response = mock.MagicMock(status_code=200)
    response.json.return_value = PINNED
    params = mock.MagicMock()
    with mock.patch('openpilot.sunnypilot.models.fetcher.requests.get', return_value=response), \
         mock.patch('openpilot.sunnypilot.accelerators.big_catalog', side_effect=lambda c, u: c) as hook:
      ModelFetcher(params)._fetch_and_cache_models(source)
    return hook

  def test_the_big_model_source_is_extended(self):
    self.fetch('chestnut').assert_called_once_with(PINNED, URL)

  def test_the_small_model_source_is_not(self):
    self.fetch('qcom').assert_not_called()


if __name__ == '__main__':
  unittest.main()
