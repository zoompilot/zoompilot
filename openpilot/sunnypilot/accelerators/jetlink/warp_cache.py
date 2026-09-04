"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The comma-side warp JIT: what builds it, and what loads it.

jetlink splits the pipeline between `warp` and `run_policy`: the warp stays on
the comma because its input is a 2 MB camera buffer already in GPU memory and
its output is the 393 KB the link has to carry anyway. That used to cost
nothing to arrange - modeld's small model carried a standalone warp JIT in its
pkl and this borrowed it.

It does not any more. Upstream fused warp and policy into one `run_model` JIT
(commaai/openpilot#38684), so the pkl has no warp to lend: `jits['run_model']`
goes camera buffer to model output in one graph, with no seam to tap. The warp
itself is still constructible - `compile_modeld.make_warp` builds the same
closure the fused JIT wraps - it just has to be JIT-compiled somewhere.

Not in modeld. That compile lands on the loader thread inside modeld's 60 s
BIG_MODEL_TIMEOUT, on a device whose only GPU the main thread is using, every
single ignition.

So it is a scons target, exactly as upstream's own dm_warp_*.pkl is: see
accelerators/SConscript, which runs compile_warp.py. launch_chffrplus.sh
builds before manager starts, so an update that moves tinygrad has a rebuilt
warp before ignition can matter. It used to be built at runtime by jetlinkd
against a hand-rolled staleness key, which was scons' dependency tracking
reimplemented inside a daemon that only runs offroad and could lose a 9 s
compile to an ignition that landed first - one drive on the small model for
the sake of a build step.

jetlinkd still builds one that is missing outright (see `ensure`), which
covers a prebuilt image made without the target. Staleness is not its problem
any more, and load_warp is what stands between a bad pickle and the car.
"""
from __future__ import annotations

import pickle
from pathlib import Path

from openpilot.common.swaglog import cloudlog

# In the source tree, where scons can write it and next to upstream's own
# dm_warp_*.pkl. The repo-wide *.pkl ignore already covers it.
#
# It used to be Paths.comma_home()/jetlink, which on AGNOS resolves under /home,
# an overlay whose upper layer is /rwtmp, a tmpfs: the pickle was gone at the
# next boot, and with the car and the comma powered together jetlinkd then lost
# the ~9 s compile race to ignition on every cold boot. That was the "no warp
# compiled yet" of the 2026-09-04 drive. The updater's reset --hard + clean
# deletes this directory, exactly as it deletes upstream's pkls, and the build
# that runs after an update puts it back before manager starts.
CACHE_DIR = Path(__file__).resolve().with_name('models')


# What TinyJit records for the keyword call below: enumerate(args) is empty and
# sorted(kwargs) gives these four, in this order.
WARP_INPUT_NAMES = ['big_frame', 'big_tfm', 'frame', 'tfm']


def call_warp(warp, tfm, big_tfm, frame, big_frame):
  """Call a warp JIT. Every caller goes through here, capture included.

  TinyJit names its inputs in _prepare_jit_inputs as `enumerate(args)` plus
  `sorted(kwargs)`, so a positional call captures [0, 1, 2, 3] and a keyword one
  captures ['big_frame', 'big_tfm', 'frame', 'tfm']. It then refuses to run a
  capture whose names differ from the call, which means the compile and every
  later call have to agree on the convention. They did not: compile_warp called
  positionally and model_state by keyword, so the JIT built offroad raised
  JitError the first time modeld ran a frame, one drive after the warp was
  built and nowhere near the code that built it. One function making both calls
  is the only way that cannot come back.
  """
  return warp(tfm=tfm, big_tfm=big_tfm, frame=frame, big_frame=big_frame)


def init_device() -> None:
  """Bring the GPU up now, on the caller's thread.

  tinygrad initialises the device on its first kernel run, not at import and
  not at load_warp - measured, it is the first call to the warp that does it -
  and that init spawns a libusb event thread. modeld runs
  config_realtime_process(7, 54) a few lines after prepare() returns, and a
  thread created after that inherits SCHED_FIFO 54 and the core-7 pin, where an
  equal-priority thread that wakes takes the core until it blocks. That is how
  this fork lost 5% of its frames once already; see
  joining._background_priority, which exists for the threads we do create.

  The device comes up either way, moments later on the loader thread. Doing it
  here is the whole difference between that libusb thread being SCHED_OTHER on
  every core and SCHED_FIFO 54 on modeld's. Failure is not worth refusing the
  accelerator over: the device will simply come up late, as it does today.
  """
  try:
    from tinygrad.tensor import Tensor
    Tensor([0.0]).realize()
  except Exception:
    cloudlog.exception("jetlink: could not bring the gpu up before modeld goes realtime")


def device_geometry() -> tuple[int, int, int, int]:
  """(cam_w, cam_h, model_w, model_h) for this device.

  The same choice modeld/SConscript makes, from the same two constants, so the
  warp jetlinkd builds is the one modeld asks for. modeld takes its camera size
  from the vipc stream instead; if the two ever disagree, load_warp raises and
  the drive runs on the small model rather than on a warp for another camera.
  """
  from openpilot.common.hardware import HARDWARE
  from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
  from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE

  camera = _os_fisheye if HARDWARE.get_device_type() == "mici" else _ar_ox_fisheye
  return camera.width, camera.height, *MEDMODEL_INPUT_SIZE


def ensure(cam_w: int, cam_h: int, model_w: int, model_h: int) -> bool:
  """Build the warp if there is none at all. Offroad only. Never raises.

  The build normally produced this (accelerators/SConscript), so on a device
  that ran one this returns True without doing anything. It earns its keep on a
  prebuilt install whose image was built without the target, where there is no
  other way to get one.

  False means modeld will fall back to the small model, which is survivable and
  the reason this does not take the daemon down with it.
  """
  if is_cached(cam_w, cam_h, model_w, model_h):
    return True
  try:
    pkl = compile_warp(cam_w, cam_h, model_w, model_h)
  except Exception:
    cloudlog.exception("jetlink: could not compile the warp")
    return False
  prune(keep={pkl})
  return True


def warp_path(cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
  return CACHE_DIR / f'warp_{cam_w}x{cam_h}_{model_w}x{model_h}_tinygrad.pkl'


def is_cached(cam_w: int, cam_h: int, model_w: int, model_h: int) -> bool:
  """Is there a warp for this geometry?

  Presence, and nothing more. Staleness is scons' job now: the target depends
  on tinygrad and on the sources that decide what gets captured, so anything
  that would invalidate the pickle rebuilds it during the build, and a stale
  one cannot outlive a build that succeeded. The one thing presence cannot
  catch - a pickle written by an incompatible tinygrad - raises in load_warp,
  which is where it has to be caught anyway.
  """
  return warp_path(cam_w, cam_h, model_w, model_h).is_file()


def compile_warp(cam_w: int, cam_h: int, model_w: int, model_h: int, out: Path | None = None) -> Path:
  """Build the warp JIT and pickle it. Offroad only: this holds the GPU.

  `out` is for scons, which names its own target; jetlinkd's fallback lets it
  default to warp_path so both writers land on the same file.

  Runs the JIT three times before pickling. TinyJit captures on the second call,
  so a pickle taken any earlier is an empty jit that silently does nothing; the
  third exercises the captured graph. Upstream's compile_dm_warp.py runs ten.
  """
  import numpy as np
  from tinygrad.device import Device
  from tinygrad.engine.jit import TinyJit
  from tinygrad.tensor import Tensor

  from openpilot.selfdrive.modeld.compile_modeld import NV12Frame, make_warp
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

  nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
  warp_jit = TinyJit(make_warp(nv12, model_w, model_h), prune=True)

  # One set of input tensors, reused: TinyJit captures against the buffers it is
  # first handed, and fresh tensors per call would capture a different graph
  # each time. Random rather than zeros so nothing can be constant-folded away.
  rng = np.random.default_rng(42)
  tfm_npy, big_tfm_npy = np.eye(3, dtype=np.float32), np.eye(3, dtype=np.float32)
  tfm = Tensor(tfm_npy, device='NPY')
  big_tfm = Tensor(big_tfm_npy, device='NPY')
  frame = Tensor.randint(nv12.size, low=0, high=256, dtype='uint8', device=Device.DEFAULT).realize()
  big_frame = Tensor.randint(nv12.size, low=0, high=256, dtype='uint8', device=Device.DEFAULT).realize()
  for _ in range(3):
    tfm_npy[:] = rng.standard_normal((3, 3)).astype(np.float32)
    big_tfm_npy[:] = rng.standard_normal((3, 3)).astype(np.float32)
    call_warp(warp_jit, tfm, big_tfm, frame, big_frame).realize()
  Device.default.synchronize()

  pkl = warp_path(cam_w, cam_h, model_w, model_h) if out is None else Path(out)
  pkl.parent.mkdir(parents=True, exist_ok=True)
  # Through a temporary: modeld reads this while jetlinkd's fallback writes it,
  # and a half-written pickle is a failed big-model load rather than a rebuild.
  tmp = pkl.with_suffix('.pkl.tmp')
  with open(tmp, 'wb') as f:
    pickle.dump(warp_jit, f)
  tmp.replace(pkl)
  cloudlog.warning("jetlink: compiled the warp for %dx%d -> %dx%d", cam_w, cam_h, model_w, model_h)
  return pkl


def load_warp(cam_w: int, cam_h: int, model_w: int, model_h: int):
  """The cached warp JIT. Raises if it is not there or is stale.

  Raising is the right answer: modeld's big-model load is already wrapped in
  the one-way fallback to the small model, and a warp we cannot trust must not
  reach the car.
  """
  if not is_cached(cam_w, cam_h, model_w, model_h):
    raise RuntimeError(f"no warp built for {cam_w}x{cam_h} -> {model_w}x{model_h}; "
                       + "the build makes it, jetlinkd rebuilds one that is missing")
  with open(warp_path(cam_w, cam_h, model_w, model_h), 'rb') as f:
    warp = pickle.load(f)

  # Two ways a pickle can load fine and still be useless, both of which have
  # happened. A JIT pickled before TinyJit captured is an empty one that
  # silently computes nothing. And one captured with a different call
  # convention raises JitError on the first frame of a drive, which is the
  # worst possible place to find out - modeld has already committed to the
  # large model by then. Both are cheap to rule out right here.
  captured = getattr(warp, 'captured', None)
  if captured is None:
    raise RuntimeError("cached warp was pickled before it captured; it computes nothing")
  names = list(getattr(captured, 'expected_names', []))
  if names != WARP_INPUT_NAMES:
    raise RuntimeError(f"cached warp expects {names}, call_warp passes {WARP_INPUT_NAMES}")
  return warp


def warm(warp, cam_w: int, cam_h: int) -> None:
  """Run a loaded warp JIT until it is cheap to call.

  Unpickling is not the expensive part of bringing the warp up. Measured on a
  comma: load_warp 0.3 s, the *first* call 1.9 s (the captured kernels are
  loaded into the GPU driver then), the second 5 ms. Whoever pays that must not
  be modeld's frame loop: the swap in JoiningModelState used to, which skipped
  ~26 camera frames and read as 16 s of modeldLagging after every join. Two
  calls, like compile_warp, against frames of the right size and identity
  transforms; nothing about the result is kept.
  """
  import numpy as np
  from tinygrad.device import Device
  from tinygrad.tensor import Tensor

  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

  size = get_nv12_info(cam_w, cam_h)[3]
  frames = [np.zeros(size, dtype=np.uint8) for _ in range(2)]
  blobs = [Tensor.from_blob(f.ctypes.data, (size,), dtype='uint8', device=Device.DEFAULT) for f in frames]
  eye = [np.eye(3, dtype=np.float32) for _ in range(2)]
  tfm, big_tfm = (Tensor(e, device='NPY').realize() for e in eye)
  for _ in range(2):
    call_warp(warp, tfm, big_tfm, blobs[0], blobs[1]).realize()
  Device.default.synchronize()


def prune(keep: set[Path]) -> None:
  """Drop warps for a geometry this device does not have.

  Only jetlinkd's fallback calls this. scons owns the files it built and
  removing one behind its back would just make it build again.
  """
  for p in CACHE_DIR.glob('warp_*_tinygrad.pkl'):
    if p not in keep:
      p.unlink(missing_ok=True)
