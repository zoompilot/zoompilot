"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The comma-side warp JIT, compiled offroad and cached.

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
single ignition. So jetlinkd does it while the car is parked and pickles the
result, which is the same shape as upstream's own `compile_dm_warp.py`: a
standalone warp JIT, built once, loaded by the process that needs it.

The cache is keyed on what the pickle actually depends on: the tinygrad the
capture was made by, and the sources that decide what was captured. See
_build_key for why that replaced the openpilot commit, and load_warp for what
is checked before a cached pickle is allowed near the car.
"""
from __future__ import annotations

import hashlib
import json
import pickle
import subprocess
from pathlib import Path

from openpilot.common.hardware import PC
from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog

# Not Paths.comma_home() on the device. That is /home/comma/.comma, and on
# AGNOS /home is an overlay whose upper layer lives in /rwtmp, a tmpfs: the
# pickle written there is gone at the next boot. With the car and the comma
# powered together, jetlinkd then loses the ~9 s compile race to ignition every
# single cold boot and modeld logs "no warp compiled yet". Found on the
# 2026-09-04 drive, where a manual offroad/onroad cycle was the only way to
# the large model. /data is the persistent partition; it is where Paths puts
# everything else that has to survive a reboot.
CACHE_DIR = Path(Paths.comma_home()) / 'jetlink' if PC else Path('/data/jetlink')


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
  """Build the warp if it is not cached. Offroad only. Never raises.

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


# What the pickle actually depends on, besides tinygrad: the graph that was
# captured, and the NV12 layout it was built around.
_WARP_SOURCES = (
  'openpilot/selfdrive/modeld/compile_modeld.py',    # make_warp, make_frame_prepare
  'openpilot/system/camerad/cameras/nv12_info.py',   # the frame layout the graph indexes
  'openpilot/sunnypilot/accelerators/jetlink/warp_cache.py',  # call_warp, and the compile itself
)


def _tinygrad_pin() -> str:
  """The tinygrad the pickle was made by, read as the submodule's pinned oid."""
  from openpilot.common.basedir import BASEDIR
  return subprocess.check_output(['git', 'rev-parse', 'HEAD:tinygrad_repo'],
                                 cwd=BASEDIR, encoding='utf8', timeout=10).strip()


def _build_key() -> str:
  """What the pickled JIT is only valid against.

  This was the openpilot commit, on the grounds that a pickled TinyJit only
  loads under the tinygrad that made it and the commit pins that submodule.
  True, but far too broad: every update to anything invalidated a warp that was
  still perfectly good, and rebuilding takes ~9 s that only jetlinkd can spend
  and only while parked. An update landing shortly before ignition then costs
  the large model for that drive. Measured that exact loss twice in one
  evening, once with 6 s between jetlinkd starting and the car going onroad,
  once with 5 s.

  So key on what the pickle actually depends on instead: the tinygrad the
  capture was made by, and the sources that decide what was captured. That is
  strictly narrower than the commit - every one of these changing implies the
  commit changed - so it cannot accept anything the old key would have
  rejected, and it stops rejecting warps for a UI or car-port change.

  Anything that fails here raises, is_cached turns that into a miss, and the
  warp is rebuilt. A wrong warp reaching the car is the one outcome that is
  not survivable, so every uncertainty resolves to a rebuild.
  """
  from openpilot.common.basedir import BASEDIR
  h = hashlib.sha256()
  h.update(_tinygrad_pin().encode())
  for rel in _WARP_SOURCES:
    h.update(rel.encode())
    h.update(hashlib.sha256((Path(BASEDIR) / rel).read_bytes()).digest())
  return h.hexdigest()


def warp_path(cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
  return CACHE_DIR / f'warp_{cam_w}x{cam_h}_{model_w}x{model_h}_tinygrad.pkl'


def _meta_path(pkl: Path) -> Path:
  return pkl.with_suffix('.json')


def is_cached(cam_w: int, cam_h: int, model_w: int, model_h: int) -> bool:
  pkl = warp_path(cam_w, cam_h, model_w, model_h)
  if not pkl.is_file():
    return False
  try:
    with open(_meta_path(pkl)) as f:
      return json.load(f).get('build') == _build_key()
  except Exception:
    return False


def compile_warp(cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
  """Build the warp JIT and pickle it. Offroad only: this holds the GPU.

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

  pkl = warp_path(cam_w, cam_h, model_w, model_h)
  pkl.parent.mkdir(parents=True, exist_ok=True)
  # Write both through temporaries: modeld reads these while we write them, and
  # a half-written pickle is a failed big-model load rather than a rebuild.
  tmp = pkl.with_suffix('.pkl.tmp')
  with open(tmp, 'wb') as f:
    pickle.dump(warp_jit, f)
  tmp.replace(pkl)
  meta_tmp = _meta_path(pkl).with_suffix('.json.tmp')
  with open(meta_tmp, 'w') as f:
    json.dump({'build': _build_key(), 'camera': [cam_w, cam_h], 'model': [model_w, model_h]}, f)
  meta_tmp.replace(_meta_path(pkl))
  cloudlog.warning("jetlink: compiled the warp for %dx%d -> %dx%d", cam_w, cam_h, model_w, model_h)
  return pkl


def load_warp(cam_w: int, cam_h: int, model_w: int, model_h: int):
  """The cached warp JIT. Raises if it is not there or is stale.

  Raising is the right answer: modeld's big-model load is already wrapped in
  the one-way fallback to the small model, and a warp we cannot trust must not
  reach the car.
  """
  if not is_cached(cam_w, cam_h, model_w, model_h):
    raise RuntimeError(f"no warp cached for {cam_w}x{cam_h} -> {model_w}x{model_h}; jetlinkd builds it offroad")
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
  """Drop warps from an older build, or for a camera this device does not have."""
  for p in CACHE_DIR.glob('warp_*_tinygrad.pkl'):
    if p not in keep:
      p.unlink(missing_ok=True)
      _meta_path(p).unlink(missing_ok=True)
