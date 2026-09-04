"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A ModelState that starts as the small model and upgrades to the Jetson.

modeld's large-model load is a one-shot: a thread with BIG_MODEL_TIMEOUT to
produce a ModelState, and a fallback to the small model that is one-way and
silent. That is the right shape for a chestnut, which is bolted to the comma
and powered from it, so it is either there when modeld asks or it is never
there at all.

A Jetson is not that. It is a separate computer on the car's ignition rail:
cranking browns it out, so its boot *starts* at roughly the moment the comma
goes onroad and takes 45 to 100 s depending on whether the last shutdown was
clean. modeld has asked and given up long before the Jetson can answer, and
the whole drive then runs on the small model with nothing wrong except the
order events happened in.

So do not make modeld wait. Hand it something that works immediately - the
small model it already loaded - and swap the Jetson in underneath when the
link, the engine and the warp are all there. modeld needs no patch for this:
it re-reads `model` every frame for the fallback it already has, and
`modelV2.big` keeps meaning exactly what it meant before.

Two rules the swap has to respect, both learned the hard way:

- **tinygrad work happens on modeld's thread.** The joining thread does link IO
  only. Building the JetlinkModelState unpickles a TinyJit and realizes
  tensors, and doing that concurrently with the small model running frames on
  the same device is not something tinygrad promises to survive. It costs a
  frame at the swap, which modeld already counts and tolerates.
- **Never swap while engaged.** The two models disagree about the world by
  ~195 m of planned path, and stepping between them is a step in the lateral
  and longitudinal targets. It also covers the swap's own cost: the warmup
  frame goes over the link and can block up to the client's FRAME_TIMEOUT, and
  a stall that long is enough dropped frames to raise modeldLagging, which is
  a soft disable. Doing it disengaged makes that a non-event. Waiting costs
  nothing either: if the driver never disengages, the drive was already going
  to be small-model.
"""
from __future__ import annotations

import os
import threading
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import drop_realtime, set_core_affinity
from openpilot.common.swaglog import cloudlog

# How long to wait before trying the link again after a join fails or the large
# model dies mid-drive. Long enough not to thrash a Jetson that is still
# booting, short enough to catch one that finished a moment later.
REJOIN_DELAY = 5.0
ENGAGEMENT_POLL_MS = 100
# How long "Big Model Loading" may keep the driver out. It is a NO_ENTRY in
# selfdrived, and while this state proxies the small model there is nothing
# else to gate it on, so without a bound a Jetson that never comes up is a
# drive that can never engage. modeld's own BIG_MODEL_TIMEOUT is the same 60 s
# a chestnut gets. The join itself keeps going after this; only the alert ends.
LOADING_TIMEOUT = 60.0


def _background_priority() -> None:
  """Get this thread off modeld's realtime core before it does anything.

  modeld runs config_realtime_process(7, 54), and a thread started after that
  inherits both the SCHED_FIFO priority and the single-core affinity from the
  thread that created it. Measured on the car with these two threads left as
  they came out of threading.Thread: modelV2 exec p50 27.9 ms, which is normal,
  against p95 90.8 and max 159.9, and 5% frame drops - enough for
  modeldLagging, which soft-disables. Under SCHED_FIFO an equal-priority thread
  that wakes takes the core until it blocks again, so a 10 Hz poll is enough to
  do that. Neither of these threads is realtime: one waits on a socket, the
  other on a USB link.
  """
  drop_realtime()
  try:
    online = os.sched_getaffinity(0)
    everything = set(range(os.cpu_count() or 1))
    if everything - online:
      set_core_affinity(sorted(everything))
  except (OSError, AttributeError):
    # PC, or a kernel that will not widen us (macOS has no affinity call at
    # all). SCHED_OTHER alone is the part that matters: the frame loop
    # preempts us wherever we end up.
    pass


class JoiningModelState:
  """Duck-types selfdrive.modeld.modeld.ModelState, with a second one inside."""

  def __init__(self, cam_w: int, cam_h: int, small, connect, build, prepare=None):
    self._small = small
    self._active = small
    self._cam = (cam_w, cam_h)
    self._connect = connect
    self._build = build

    # Whatever the swap would otherwise have to do on the frame loop, done now
    # instead. modeld constructs this from its loader thread and waits for it
    # on the main thread, so the GPU is idle and no frame can be dropped;
    # measured on the car, doing it at the swap cost a 1.5 s frame. Failing
    # here is survivable: build() then does the work at the swap, slowly.
    if prepare is not None:
      try:
        t0 = time.monotonic()
        prepare()
        cloudlog.warning("jetlink: prepared the large model ahead of the swap in %.2f s", time.monotonic() - t0)
      except Exception:
        cloudlog.exception("jetlink: could not prepare the large model ahead of the swap")

    # Handed over by the joining thread, consumed by whichever modeld frame
    # first finds it safe to swap. Only ever assigned under the lock.
    self._joined: tuple[object, object] | None = None
    self._lock = threading.Lock()
    self._rejoin = threading.Event()
    self._rejoin.set()
    # Earliest the join loop may try again. A demote used to set _rejoin alone,
    # so the retry started on the very next frame and a fault that recurred
    # on the first frame after every swap became a connect, build, warmup and
    # demote every 700 ms for the whole drive, each swap costing modeld a
    # 100 ms frame. That was the transport desync of 2026-09-03, since fixed
    # in jetlink; the backoff is here so the next one is a slow leak and not
    # modeldLagging.
    self._rejoin_at = 0.0

    # Assume engaged until a message says otherwise, so a swap can never happen
    # on no information. selfdrived publishes at 100 Hz, so this is true within
    # a frame or two of modeld starting.
    self._engaged = True
    self._stop = threading.Event()

    # The UI reads ChestnutLoading to tell "not up yet" from "failed". modeld
    # clears it as soon as make_model_state returns, which for us is before the
    # Jetson has joined, so we own it from here: true while proxying, false once
    # the large model is running, true again if it drops and we go back for it.
    self._params = Params()
    self._loading = False
    self._loading_until = time.monotonic() + LOADING_TIMEOUT

    self._threads = [threading.Thread(target=self._join_loop, daemon=True),
                     threading.Thread(target=self._watch_engagement, daemon=True)]
    for t in self._threads:
      t.start()

  # -- what modeld reads ------------------------------------------------------

  @property
  def chestnut(self) -> bool:
    # modelV2.big. False while proxying, which is what the small model would
    # have reported anyway, and the signal the docs tell you to trust.
    return getattr(self._active, 'chestnut', False)

  @property
  def vision_input_names(self):
    return self._active.vision_input_names

  @property
  def client(self):
    # The health publisher reads this every time it sends, so chestnutState
    # starts reporting the moment the Jetson joins and stops if it leaves.
    return getattr(self._active, 'client', None)

  @property
  def lat_delay(self):
    return self._active.lat_delay

  @lat_delay.setter
  def lat_delay(self, value):
    # modeld writes this every frame. Set it on both, so a model that joins
    # mid-drive does not run its first frames on a stale delay.
    self._small.lat_delay = value
    if self._active is not self._small:
      self._active.lat_delay = value

  # -- the frame path ---------------------------------------------------------

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    # Before the first modelV2 goes out, so the UI never sees a published frame
    # with big=false while it still thinks the load finished.
    self._set_loading(self._active is self._small and time.monotonic() < self._loading_until)
    self._maybe_swap()
    active = self._active
    try:
      return active.run(bufs, transforms, inputs, after_enqueue)
    except Exception:
      if active is self._small:
        # Nothing to do with the link. modeld's own handler owns this.
        raise
      cloudlog.exception("jetlink: large model failed mid-drive, back to the small model")
      self._demote()
      # Re-run the frame rather than propagate. modeld's fallback would take the
      # same decision and make it permanent; this one is retryable, which is
      # what a Jetson that rebooted or a cable that was nudged actually needs.
      # after_enqueue is dropped: the large model may already have called it,
      # and chestnutState is published once per frame at most.
      return self._small.run(bufs, transforms, inputs, None)

  def warmup(self) -> None:
    """modeld warms whatever make_model_state returned. Nothing to do here.

    The small model is already warm: modeld built and warmed it on the main
    thread before this object existed. The Jetson's model is warmed at the
    swap instead, on modeld's thread, which is where the tinygrad work has to
    happen anyway. So this is a no-op, but it has to exist - modeld calls it
    unconditionally, and an AttributeError is caught as "big model load failed"
    and costs the whole drive.
    """

  def _set_loading(self, loading: bool) -> None:
    if loading != self._loading:
      self._loading = loading
      self._params.put_bool("ChestnutLoading", loading)

  def _maybe_swap(self) -> None:
    if self._joined is None or self._engaged:
      return
    with self._lock:
      joined, self._joined = self._joined, None
    if joined is None:
      return
    client, spec = joined
    try:
      # Everything tinygrad touches happens here, on modeld's own thread. No
      # warmup: with the warp prepared in __init__ the first real frame is the
      # cheapest warm-up there is (its reset costs the server ~30 ms), where a
      # warmup frame over the link was two more dropped frames for nothing.
      t0 = time.monotonic()
      big = self._build(client, spec)
      big.lat_delay = self._small.lat_delay
      cloudlog.warning("jetlink: built the large model state in %.0f ms", (time.monotonic() - t0) * 1000)
    except Exception:
      cloudlog.exception("jetlink: could not bring up the large model, staying small")
      try:
        client.close()
      except Exception:
        pass
      self._rejoin.set()
      return
    self._active = big
    self._set_loading(False)
    cloudlog.warning("jetlink: large model joined mid-drive, modelV2.big is now true")

  def _demote(self) -> None:
    big, self._active = self._active, self._small
    self._loading_until = time.monotonic() + LOADING_TIMEOUT
    self._set_loading(True)
    close = getattr(big, 'close', None)
    if close is not None:
      try:
        close()
      except Exception:
        cloudlog.exception("jetlink: closing the failed large model")
    self._rejoin_at = time.monotonic() + REJOIN_DELAY
    self._rejoin.set()

  # -- background -------------------------------------------------------------

  def _join_loop(self) -> None:
    """Open the link and get the engine ready. No tinygrad in here."""
    _background_priority()
    while not self._stop.is_set():
      # No timeout: once joined there is nothing to poll for, and close() sets
      # this to wake us. An idle wake per second is not free on modeld's core.
      self._rejoin.wait()
      if self._stop.is_set():
        return
      self._rejoin.clear()
      if self._stop.wait(max(0.0, self._rejoin_at - time.monotonic())):
        return
      try:
        client, spec = self._connect()
      except Exception as e:
        # Expected while the Jetson boots. Not exception(): a stack trace every
        # 5 s for the first minute of every drive is noise, not a signal.
        cloudlog.warning("jetlink: not joined yet (%s), retrying in %.0fs", e, REJOIN_DELAY)
        if self._stop.wait(REJOIN_DELAY):
          return
        self._rejoin.set()
        continue
      with self._lock:
        self._joined = (client, spec)
      cloudlog.warning("jetlink: link ready, waiting for a disengaged frame to swap")

  def _watch_engagement(self) -> None:
    _background_priority()
    sm = messaging.SubMaster(['selfdriveState'])
    while not self._stop.is_set():
      sm.update(ENGAGEMENT_POLL_MS)
      if sm.updated['selfdriveState']:
        self._engaged = sm['selfdriveState'].enabled

  def close(self) -> None:
    self._stop.set()
    self._rejoin.set()
    with self._lock:
      joined, self._joined = self._joined, None
    if joined is not None:
      joined[0].close()
    close = getattr(self._active, 'close', None)
    if close is not None and self._active is not self._small:
      close()
