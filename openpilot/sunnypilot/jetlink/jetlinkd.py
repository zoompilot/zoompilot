#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Provisions the Jetson with whatever large model is selected.

Runs offroad, like the model downloader, and for the same reason: uploading
766 MB and building a TensorRT engine takes minutes, which is far longer than
modeld's 60 s big-model timeout. By the time the car goes onroad the engine is
built and cached, and modeld only has to load it (~1 s).

It also means only one process touches the link at a time: jetlinkd offroad,
modeld onroad. That is enforced by the `only_offroad` gate in process_config,
which manager applies - there is no onroad param to consult, and checking one
here would only duplicate it.
"""
from __future__ import annotations

import time

from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.jetlink import helpers, spec_cache

POLL_HZ = 0.5
RETRY_BACKOFF = 30.0


def provision_once() -> bool:
  """Returns True when the Jetson is ready for the selected model."""
  model_path = helpers.active_model_path()
  if model_path is None:
    # Nothing selected, or still downloading. Not an error.
    helpers.set_engine_ready(None)
    helpers.clear_progress()
    return False

  # Steady state must not touch the model file. Hashing 766 MB takes longer
  # than this loop's period, so doing it per poll would peg a core for as long
  # as the car is parked. If the cached spec came from this exact file and its
  # engine is ready, there is nothing to do.
  st = model_path.stat()
  source = (str(model_path), st.st_mtime_ns, st.st_size)
  spec = spec_cache.load()
  if (spec_cache.source() == source and spec is not None
      and helpers.engine_ready_for(spec.sha256)):
    return True

  from jetlink.spec import spec_from_onnx
  spec = spec_from_onnx(str(model_path))

  if helpers.engine_ready_for(spec.sha256):
    spec_cache.store(spec, model_path)   # record the source so the next poll is free
    return True

  cloudlog.warning("jetlink: provisioning %s (%d MB, sha %s)",
                   model_path.name, spec.nbytes >> 20, spec.sha256[:16])
  helpers.report_progress('connect', 0.0, 'connecting to jetson')

  client = helpers.connect(deadline=5.0)
  try:
    hello = client.hello(timeout=10.0)
    cloudlog.warning("jetlink: server %s trt %s", hello.get('device'), hello.get('trt_version'))
    client.ensure_engine(model_path, spec=spec, progress=helpers.report_progress,
                         build_timeout=1800.0)
  finally:
    client.close()

  spec_cache.store(spec, model_path)
  helpers.set_engine_ready(spec.sha256)
  helpers.report_progress('ready', 1.0, 'engine ready')
  helpers.cleanup_unchunked(keep=model_path)
  cloudlog.warning("jetlink: engine ready for %s", spec.sha256[:16])
  return True


def main() -> None:
  rk = Ratekeeper(POLL_HZ)
  next_attempt = 0.0
  ready = False

  while True:
    rk.keep_time()

    try:
      if not helpers.enabled():
        if ready:
          cloudlog.warning("jetlink: link gone, clearing readiness")
          helpers.set_engine_ready(None)
          ready = False
        continue
      if time.monotonic() < next_attempt:
        continue

      ready = provision_once()
      next_attempt = 0.0
    except Exception:
      # Nothing here may escape: this daemon restarting in a loop would be
      # worse than it sitting out a provisioning cycle.
      cloudlog.exception("jetlink: provisioning failed")
      helpers.report_progress('failed', 1.0, 'see the log')
      helpers.set_engine_ready(None)
      ready = False
      next_attempt = time.monotonic() + RETRY_BACKOFF


if __name__ == "__main__":
  main()
