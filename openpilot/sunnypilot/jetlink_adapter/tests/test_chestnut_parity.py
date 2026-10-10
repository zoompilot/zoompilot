"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A chestnut on this branch behaves as it does on sunnypilot, except where
zoompilot does it better: the link stays off beside it, its failure is
upstream's own event and text, and a pick whose files are missing is kept
while the Default big model drives.
"""
from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest import mock

from jetlink.comma import gadget

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, EventName
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.models import helpers
from openpilot.sunnypilot.models.fetcher import ModelParser

REF = 'b' * 40


class TestAlert(OpenpilotTestCase):
  def test_a_chestnut_is_told_to_restart_as_upstream_says(self):
    # only a chestnut raises it; an accelerator's loss is bigModelLinkLost,
    # a warning with the small model driving on (accelerator_events)
    alert = EVENTS[EventName.bigModelFailed][ET.PERMANENT]
    self.assertEqual(alert.alert_text_2, "Restart the car to retry,\nsmall model is still available")


class TestLinkStaysOff(OpenpilotTestCase):
  """Through the adapter, as manager, hardwared and the UI ask, with the link set
  to USB and a gadget that cannot come up."""

  def setUp(self):
    super().setUp()
    Params().put(jetlink_adapter.KEYS.link, jetlink_adapter.MODES.index('usb'), block=True)
    for p in (mock.patch.object(gadget, 'gadget_error', return_value='not set up'),
              mock.patch.object(jetlink_adapter, '_bound', None)):
      p.start()
      self.addCleanup(p.stop)

  def fitted(self, chestnut: bool):
    jetlink_adapter._bound = None   # a process of its own: the bus walk is cached
    return mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=chestnut)

  def test_the_setting_on_beside_a_chestnut_is_off(self):
    with self.fitted(True):
      self.assertFalse(jetlink_adapter.should_run(False, None, None))
      status = jetlink_adapter.status()
      self.assertFalse(status.enabled or status.ready)
      self.assertIsNone(status.reason)
      self.assertIsNone(jetlink_adapter.reason())
      # jetlink's own answer too, had modeld missed the chestnut
      self.assertFalse(jetlink_adapter.prepare(False))

  def test_without_one_the_setting_decides(self):
    with self.fitted(False):
      self.assertTrue(jetlink_adapter.should_run(False, None, None))
      self.assertEqual(jetlink_adapter.reason(), 'not set up')
      self.assertEqual(jetlink_adapter.status().reason, 'not set up')

  def test_the_bus_walk_is_cached(self):
    with self.fitted(True) as probe:
      for _ in range(5):
        jetlink_adapter.should_run(False, None, None)
        jetlink_adapter.status()
    self.assertEqual(probe.call_count, 1)


class TestPickKeptDefaultDrives(OpenpilotTestCase):
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

  def active(self):
    return helpers.get_active_bundle(self.store, chestnut=True)

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


if __name__ == '__main__':
  unittest.main()
