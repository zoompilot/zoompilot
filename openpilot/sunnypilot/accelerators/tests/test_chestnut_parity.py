"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A chestnut on this branch behaves as it does on sunnypilot, except where
zoompilot does it better: the link stays off beside it, its failure says to
restart as upstream's does, and a pick whose files are missing is kept while
the Default big model drives.
"""
from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest import mock

from openpilot.selfdrive.selfdrived.events import big_model_failed_alert
from openpilot.sunnypilot import accelerators
from openpilot.sunnypilot.accelerators.jetlink import backend, gadget, helpers as jl_helpers
from openpilot.sunnypilot.models import helpers
from openpilot.sunnypilot.models.fetcher import ModelParser

REF = 'b' * 40


def failed_text(chestnut: bool) -> str:
  sm = {'deviceState': SimpleNamespace(chestnutPresent=chestnut)}
  return big_model_failed_alert(None, None, sm, True, 0, None).alert_text_2


class TestAlert(unittest.TestCase):
  def test_a_chestnut_is_told_to_restart_as_upstream_says(self):
    self.assertEqual(failed_text(True), "Restart the car to retry,\nsmall model is still available")

  def test_an_accelerator_is_not(self):
    self.assertEqual(failed_text(False), "Small model is still available")


class TestLinkStaysOff(unittest.TestCase):
  def setUp(self):
    backend._chestnut = None
    self.addCleanup(setattr, backend, '_chestnut', None)
    for p in (mock.patch.object(gadget, 'raw_param', return_value=b'1'),
              mock.patch.object(gadget, 'gadget_error', return_value='not set up')):
      p.start()
      self.addCleanup(p.stop)

  def fitted(self, chestnut: bool):
    backend._chestnut = None
    return mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=chestnut)

  def test_the_toggle_on_beside_a_chestnut_is_off(self):
    with self.fitted(True):
      self.assertFalse(accelerators.enabled())
      self.assertFalse(accelerators.ready())
      self.assertIsNone(accelerators.unavailable_reason())
      (daemon,) = accelerators.daemons()
      self.assertFalse(daemon.should_run(False, None, None))

  def test_without_one_the_toggle_decides(self):
    with self.fitted(False):
      self.assertTrue(accelerators.enabled())
      self.assertEqual(accelerators.unavailable_reason(), jl_helpers.gadget_alert())

  def test_the_bus_walk_is_cached(self):
    with self.fitted(True) as probe:
      for _ in range(5):
        accelerators.enabled()
    self.assertEqual(probe.call_count, 1)


class TestPickKeptDefaultDrives(unittest.TestCase):
  """A chestnut pick whose files are not here runs as the Default big model."""

  def setUp(self):
    import tempfile
    self.root = tempfile.mkdtemp()
    bundle = ModelParser._parse_bundle({
      'index': 13, 'short_name': 'CTV3M', 'display_name': 'Cinque Terre V3', 'generation': '12',
      'environment': 'development', 'runner': 'tinygrad', 'is_20hz': True, 'ref': REF,
      'minimum_selector_version': str(helpers.REQUIRED_JSON_VERSION),
      'models': [{'type': 'supercombo', 'artifact': {'file_name': 'ctv3.pkl', 'download_uri': {'url': 'x', 'sha256': 's'}}}],
    })
    self.params = {helpers.ACTIVE_BUNDLE_KEYS['chestnut']: bundle.to_dict()}
    store = SimpleNamespace(get=lambda k, *a, **kw: self.params.get(k))
    for p in (mock.patch.object(helpers.Paths, 'model_root', return_value=self.root),):
      p.start()
      self.addCleanup(p.stop)
    self.store = store

  def active(self, chestnut=True):
    return helpers.get_active_bundle(self.store, chestnut=chestnut)

  def test_missing_files_drive_the_default_and_keep_the_pick(self):
    self.assertIsNone(self.active())
    self.assertIn(helpers.ACTIVE_BUNDLE_KEYS['chestnut'], self.params)

  def test_the_pick_drives_once_its_files_are_here(self):
    import os
    open(os.path.join(self.root, 'ctv3.pkl'), 'wb').close()
    self.assertEqual(self.active().ref, REF)

  def test_a_pick_being_fetched_again_drives_the_default(self):
    import os
    open(os.path.join(self.root, 'ctv3.pkl'), 'wb').close()
    self.params['ModelManager_DownloadRef'] = REF
    self.assertIsNone(self.active())

  def test_without_a_chestnut_the_small_slot_is_what_runs(self):
    self.params[helpers.ACTIVE_BUNDLE_KEYS['qcom']] = None
    self.assertIsNone(self.active(chestnut=False))


if __name__ == '__main__':
  unittest.main()
