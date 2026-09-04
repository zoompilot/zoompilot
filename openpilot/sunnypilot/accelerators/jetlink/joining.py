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
- **Never swap while the plan is steering the car.** The two models disagree
  about the world by ~195 m of planned path, and stepping between them is a
  step in the lateral and longitudinal targets. It also covers the swap's own
  cost: the warmup frame goes over the link and can block up to the client's
  FRAME_TIMEOUT, and a stall that long is enough dropped frames to raise
  modeldLagging, which is a soft disable.

  Disengaged is the obvious window. Standstill is the other one, and it is the
  one that makes this usable: a driver who engages on the ramp and lifts off at
  their exit gives us no disengaged frame for the whole drive, and "the Jetson
  was ready the whole time and never got used" is the complaint that shape of
  drive produces. At a standstill the plan is not turning a wheel or asking for
  acceleration, so the step lands on nothing. A drive that is neither - engaged
  from the driveway to the destination without ever stopping - still runs
  small, on purpose.
"""
from __future__ import annotations

import os
import threading
import time

import openpilot.cereal.messaging as messaging
from openpilot.sunnypilot import accelerators
from openpilot.common.params import Params
from openpilot.common.realtime import drop_realtime, set_core_affinity
from openpilot.common.swaglog import cloudlog

# How long to wait before trying the link again after a join fails or the large
# model dies mid-drive. Long enough not to thrash a Jetson that is still
# booting, short enough to catch one that finished a moment later.
REJOIN_DELAY = 5.0
# Doubled per consecutive failure up to this, and reset by a join that lasted
# STABLE_SECONDS. A Jetson that reboots mid-drive is still picked up within a
# minute of being back; a link that dies on its first frame every time stops
# costing a swap, a demote and an alert every few seconds.
REJOIN_DELAY_MAX = 60.0
STABLE_SECONDS = 60.0
ENGAGEMENT_POLL_MS = 100
# How often a link that is ready but has nowhere to land gets checked, and how
# long its check may take. Both are off the frame loop.
KEEPALIVE_PERIOD = 10.0
PING_TIMEOUT = 2.0


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
    self._failures = 0
    self._joined_at = 0.0

    # Assume engaged and moving until a message says otherwise, so a swap can
    # never happen on no information. selfdrived and the car both publish at
    # 100 Hz, so this is true within a frame or two of modeld starting.
    self._engaged = True
    self._standstill = False
    self._stop = threading.Event()

    # The two params modeld normally writes once the load is over are ours for
    # the life of the drive, because for us the load is never over: the Jetson
    # can join, leave and join again. modeld reads `loading` and leaves both
    # alone while it is true.
    #
    # ChestnutLoading is true while this proxies and false while the large
    # model runs. selfdrived rings "Big Model Ready" on its falling edge, so
    # that lands on the swap and nowhere else, and it only holds the driver out
    # while nothing is publishing modelV2, which for us is never. It used to
    # be bounded at 60 s because it was a NO_ENTRY: a Jetson that took longer
    # than that was a drive that could not engage, and the timeout read as
    # "ready" to selfdrived and "unavailable" to the UI, both false, while
    # this thread was still joining.
    #
    # ChestnutActive is absent while proxying (a big model that is neither
    # active nor failed: selfdrived and the UI both take None as "still
    # coming"), true at the swap, false at a demote. False is what a chestnut
    # writes when it dies mid-drive and gets the same soft disable: the two
    # models plan ~195 m apart and the driver should know the plan just
    # changed under them. Re-engaging on the small model is allowed at once.
    self._params = Params()
    self._loading: bool | None = None
    self._params.remove("ChestnutActive")
    self._set_loading(True)

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
  def loading(self) -> bool:
    # Still bringing the accelerator up. modeld reads this once, after the
    # load, to know it must not write ChestnutLoading and ChestnutActive itself.
    return self._active is self._small

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

  def _set_active(self, active: bool) -> None:
    # Before loading goes false at the swap, so that when selfdrived sees the
    # ready edge the active flag it reads in the same tick is already true.
    self._params.put_bool("ChestnutActive", active)

  @property
  def _window_open(self) -> bool:
    # See the module docstring: disengaged, or engaged but stopped.
    return not self._engaged or self._standstill

  def _maybe_swap(self) -> None:
    if self._joined is None or not self._window_open:
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
      # Backed off like a demote: a build that fails the same way every time
      # would otherwise be a connect and a build per second for the drive.
      self._back_off()
      return
    self._active = big
    self._joined_at = time.monotonic()
    self._set_active(True)
    self._set_loading(False)
    accelerators.clear_progress()
    cloudlog.warning("jetlink: large model joined mid-drive, modelV2.big is now true")

  def _demote(self) -> None:
    big, self._active = self._active, self._small
    self._set_active(False)
    self._set_loading(True)
    self._report('connect', 'lost the jetson, reconnecting')
    close = getattr(big, 'close', None)
    if close is not None:
      try:
        close()
      except Exception:
        cloudlog.exception("jetlink: closing the failed large model")
    self._back_off()

  def _back_off(self) -> None:
    """Push the next attempt out, and further each time one fails on its heels.

    A link that dies the same way every time it comes up is the shape that
    costs the most: each cycle is a swap frame, a demote frame, a soft disable
    and a "Big Model Ready" chime, and at a flat delay it repeats for the whole
    drive. A join that held for STABLE_SECONDS was not that, and starts the
    next one from the bottom again.
    """
    stable = self._joined_at and time.monotonic() - self._joined_at > STABLE_SECONDS
    self._failures = 1 if stable else self._failures + 1
    self._joined_at = 0.0
    delay = min(REJOIN_DELAY * 2 ** (self._failures - 1), REJOIN_DELAY_MAX)
    self._rejoin_at = time.monotonic() + delay
    self._rejoin.set()
    cloudlog.warning("jetlink: next attempt in %.0f s (failure %d)", delay, self._failures)

  # -- background -------------------------------------------------------------

  def _report(self, stage: str, msg: str) -> None:
    """Tell the UI what the join is waiting on.

    The models panel already renders this param, and offroad it carries
    jetlinkd's provisioning. Onroad nothing was writing it, so a join showed
    as "getting ready" and nothing else - a Jetson that is not plugged in
    looked exactly like one whose engine is six seconds from loading. There
    is no fraction to give here, and the panel knows not to invent one.
    """
    accelerators.report_progress(stage, 0.0, msg)

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
      self._report('connect', 'waiting for the jetson')
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
      cloudlog.warning("jetlink: link ready, waiting for a window to swap")
      self._report('connect', 'ready, waiting for a safe moment')
      self._keep_alive()

  def _keep_alive(self) -> None:
    """Keep a link that has nowhere to land honest until it can be used.

    A ready link waits for the frame loop to find a swap window, and on a drive
    where the driver never stops and never lifts off that is the whole drive.
    A Jetson that reboots inside that window leaves a dead client parked in
    _joined, and the swap is where we would find out: a build on a dead link,
    a demote, and the backoff, all on modeld's thread. Ping it here instead and
    start the rejoin now, so what the window finally opens onto is a link that
    answered a moment ago.

    The client is taken out of _joined for the ping and put back after, so the
    frame loop either sees a whole one or sees none. It never blocks on the
    lock waiting for a ping to finish, because _maybe_swap does not take the
    lock at all when _joined is None.
    """
    while not self._stop.wait(KEEPALIVE_PERIOD):
      with self._lock:
        joined, self._joined = self._joined, None
      if joined is None:
        return  # swapped in on a frame, or closed under us
      try:
        joined[0].ping(timeout=PING_TIMEOUT)
      except Exception as e:
        cloudlog.warning("jetlink: the link died before it could be used (%s), reopening", e)
        try:
          joined[0].close()
        except Exception:
          pass
        self._rejoin_at = time.monotonic() + REJOIN_DELAY
        self._rejoin.set()
        return
      with self._lock:
        if self._stop.is_set():
          # close() ran while we held it, and found nothing to close.
          joined[0].close()
          return
        self._joined = joined

  def _watch_engagement(self) -> None:
    _background_priority()
    sm = messaging.SubMaster(['selfdriveState', 'carState'])
    while not self._stop.is_set():
      sm.update(ENGAGEMENT_POLL_MS)
      if sm.updated['selfdriveState']:
        self._engaged = sm['selfdriveState'].enabled
      if sm.updated['carState']:
        # Only believed while the car is actually talking to us. A carState
        # that stopped arriving must not read as "stopped" and open a window
        # that is not there.
        self._standstill = sm.alive['carState'] and sm['carState'].standstill

  def close(self) -> None:
    self._stop.set()
    self._rejoin.set()
    accelerators.clear_progress()
    with self._lock:
      joined, self._joined = self._joined, None
    if joined is not None:
      joined[0].close()
    close = getattr(self._active, 'close', None)
    if close is not None and self._active is not self._small:
      close()
