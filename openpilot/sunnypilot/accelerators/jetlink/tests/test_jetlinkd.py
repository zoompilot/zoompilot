"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

What jetlinkd does when the far end is attached but not serving.

That is the expensive state, not the one where no Jetson is plugged in: the
daemon has a host to talk to and keeps trying, so anything it repeats per
attempt it repeats for as long as the car is parked.
"""

import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.accelerators.jetlink import jetlinkd


class FakeSpec:
  def __init__(self, sha256: str = 'deadbeef', nbytes: int = 1 << 20):
    self.sha256 = sha256
    self.nbytes = nbytes


class FakeSpecCache:
  """spec_cache backed by memory rather than a param."""

  def __init__(self):
    self.spec = None
    self.src = None
    self.stores = 0

  def load(self):
    return self.spec

  def source(self):
    return self.src

  def store(self, spec, source: Path | None = None) -> None:
    self.stores += 1
    self.spec = spec
    if source is not None:
      st = source.stat()
      self.src = (str(source), st.st_mtime_ns, st.st_size)


def fake_jetlink_spec_module(counter: list):
  """A stand-in for jetlink.spec, which is not importable without the package."""
  mod = types.ModuleType('jetlink.spec')

  def sha256_file(path, *a, **kw):
    counter.append(path)
    return 'deadbeef', 1 << 20

  mod.sha256_file = sha256_file
  return {'jetlink': types.ModuleType('jetlink'), 'jetlink.spec': mod}


def serving_client(spec=None):
  """A client whose server already has the engine."""
  client = mock.Mock()
  client.ensure_engine.return_value = spec or FakeSpec()
  return client


class TestProvisionCost(unittest.TestCase):
  def setUp(self):
    self.model = Path(tempfile.mkdtemp()) / 'big_driving_supercombo.onnx'
    self.model.write_bytes(b'x' * 4096)
    self.cache = FakeSpecCache()
    self.hashed: list[str] = []

    for target, new in (('spec_cache', self.cache), ('accelerators', mock.Mock())):
      p = mock.patch.object(jetlinkd, target, new)
      self.addCleanup(p.stop)
      p.start()
    for name, value in (('active_model_path', self.model), ('engine_ready_for', False)):
      p = mock.patch.object(jetlinkd.helpers, name, return_value=value)
      self.addCleanup(p.stop)
      p.start()
    p = mock.patch.dict(sys.modules, fake_jetlink_spec_module(self.hashed))
    self.addCleanup(p.stop)
    p.start()

  def test_the_model_is_hashed_once_however_often_provisioning_fails(self):
    # The whole point: hashing is 766 MB-1.8 GB of flash reads, and an engine
    # that never gets built must not make that the per-retry cost.
    d = jetlinkd.Jetlinkd()
    d.client = mock.Mock()
    d.client.hello.side_effect = RuntimeError('nothing is serving')
    for _ in range(3):
      with self.assertRaises(RuntimeError):
        d.provision()
    assert self.hashed == [str(self.model)]
    # The shapes come from the server, so there is nothing to cache until it
    # answers; the identity is what must survive the retries, and it did.
    assert self.cache.stores == 0

  def test_a_changed_model_is_hashed_again(self):
    d = jetlinkd.Jetlinkd()
    d.client = mock.Mock()
    d.client.hello.side_effect = RuntimeError('nothing is serving')
    with self.assertRaises(RuntimeError):
      d.provision()
    self.model.write_bytes(b'y' * 8192)
    with self.assertRaises(RuntimeError):
      d.provision()
    assert len(self.hashed) == 2

  def test_a_ready_engine_short_circuits_without_touching_the_file(self):
    self.cache.store(FakeSpec(), self.model)
    d = jetlinkd.Jetlinkd()
    d.verified = True
    with mock.patch.object(jetlinkd.helpers, 'engine_ready_for', return_value=True):
      assert d.provision() is True
    assert self.hashed == []

  def test_a_ready_param_is_checked_with_the_server_once_per_attach(self):
    # The Jetson's cache can be pruned, re-flashed or swapped under a param
    # that says ready; trusting it blindly used to cost every drive until
    # someone cleared the param by hand.
    self.cache.store(FakeSpec(), self.model)
    d = jetlinkd.Jetlinkd()
    d.client = serving_client()
    with mock.patch.object(jetlinkd.helpers, 'engine_ready_for', return_value=True), \
         mock.patch.object(jetlinkd.helpers, 'set_engine_ready') as ready:
      assert d.provision() is True
      assert d.client.ensure_engine.call_count == 1
      assert d.verified
      assert d.provision() is True
      assert d.client.ensure_engine.call_count == 1, "verified once, then the param is trusted"
    assert self.hashed == []
    ready.assert_called_with('deadbeef')

  def test_the_shapes_come_from_the_server_not_the_file(self):
    d = jetlinkd.Jetlinkd()
    d.client = serving_client(FakeSpec(sha256='deadbeef', nbytes=1 << 20))
    with mock.patch.object(jetlinkd.helpers, 'set_engine_ready'):
      assert d.provision() is True
    d.client.ensure_engine.assert_called_once()
    kwargs = d.client.ensure_engine.call_args.kwargs
    assert kwargs['onnx_path'] == self.model
    assert callable(kwargs['should_stop'])
    assert self.cache.stores == 1 and self.cache.spec.sha256 == 'deadbeef'

  def test_stop_is_polled_through_the_long_wait(self):
    d = jetlinkd.Jetlinkd()
    d.client = serving_client()
    with mock.patch.object(jetlinkd.helpers, 'set_engine_ready'):
      d.provision()
    should_stop = d.client.ensure_engine.call_args.kwargs['should_stop']
    assert should_stop() is False
    d.request_stop()
    assert should_stop() is True


class TestBackoff(unittest.TestCase):
  def test_it_doubles_and_then_stops(self):
    d = jetlinkd.Jetlinkd()
    seen = []
    for _ in range(8):
      d.failures += 1
      seen.append(d.backoff())
    assert seen[:5] == [30.0, 60.0, 120.0, 240.0, 480.0]
    assert seen[-1] == jetlinkd.RETRY_BACKOFF_MAX


class TestStepOnFailure(unittest.TestCase):
  def _step(self, d, provision):
    for p in (mock.patch.object(jetlinkd, 'accelerators', mock.Mock()),
              mock.patch.object(jetlinkd.helpers, 'enabled', return_value=True),
              mock.patch.object(jetlinkd.helpers, 'host_attached', return_value=True),
              mock.patch.object(d, 'open_link', return_value=True),
              mock.patch.object(d, 'provision', provision),
              mock.patch.object(d, 'close_link', mock.Mock())):
      self.addCleanup(p.stop)
      p.start()
    d.step()

  def test_a_timeout_keeps_the_gadget_presented(self):
    # Unbinding makes the host re-enumerate. Doing that on every retry is what
    # produced hours of connect/disconnect against a Jetson that was simply
    # not running the server.
    d = jetlinkd.Jetlinkd()
    d.client = object()
    with mock.patch.object(jetlinkd, '_timed_out', return_value=True):
      self._step(d, mock.Mock(side_effect=RuntimeError('no reply')))
    assert d.close_link.call_count == 0
    assert d.failures == 1
    assert d.next_provision > time.monotonic()

  def test_anything_else_reopens_the_link(self):
    d = jetlinkd.Jetlinkd()
    d.client = object()
    with mock.patch.object(jetlinkd, '_timed_out', return_value=False):
      self._step(d, mock.Mock(side_effect=RuntimeError('desynced')))
    assert d.close_link.call_count == 1
    assert d.failures == 1

  def test_a_host_that_arrives_does_not_serve_out_the_old_backoff(self):
    d = jetlinkd.Jetlinkd()
    d.failures = 6
    d.next_provision = time.monotonic() + jetlinkd.RETRY_BACKOFF_MAX
    provision = mock.Mock(return_value=True)
    self._step(d, provision)
    assert provision.call_count == 1
    assert d.failures == 0
    assert d.ready is True


class TestTimedOut(unittest.TestCase):
  def test_without_the_package_it_assumes_the_worst(self):
    # No jetlink installed means no way to tell a timeout from a desync, and
    # reopening a healthy link is cheaper than reusing a broken one.
    assert jetlinkd._timed_out(RuntimeError('boom')) is False

  def test_it_follows_jetlink_own_distinction(self):
    base = types.ModuleType('jetlink.transport.base')

    class LinkError(IOError):
      pass

    class LinkTimeout(LinkError):
      pass

    base.LinkError, base.LinkTimeout = LinkError, LinkTimeout
    mods = {'jetlink': types.ModuleType('jetlink'),
            'jetlink.transport': types.ModuleType('jetlink.transport'),
            'jetlink.transport.base': base}
    with mock.patch.dict(sys.modules, mods):
      assert jetlinkd._timed_out(LinkTimeout('no reply in time')) is True
      assert jetlinkd._timed_out(LinkError('stream desynced')) is False


if __name__ == '__main__':
  unittest.main()
