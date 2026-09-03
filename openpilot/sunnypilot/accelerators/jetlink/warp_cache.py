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

The cache is keyed on the openpilot commit rather than on anything finer. A
pickled TinyJit is only loadable by the tinygrad that made it, tinygrad is a
submodule, and the commit pins the submodule - so the coarse key is the correct
one, and it costs a rebuild on update, parked, once.
"""
from __future__ import annotations

import json
import pickle
from pathlib import Path

from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog

CACHE_DIR = Path(Paths.comma_home()) / 'jetlink'


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


def _build_key() -> str:
  """What the pickled JIT is only valid against.

  The openpilot commit, because tinygrad is a submodule it pins: a pickle from
  another tinygrad will not load, and one that half-loads is worse. Coarse on
  purpose - the cost of being wrong here is a silent fallback to the small
  model mid-drive, and the cost of being conservative is one parked rebuild.
  """
  from openpilot.common.version import get_build_metadata
  return get_build_metadata().openpilot.git_commit


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
    return pickle.load(f)


def prune(keep: set[Path]) -> None:
  """Drop warps from an older build, or for a camera this device does not have."""
  for p in CACHE_DIR.glob('warp_*_tinygrad.pkl'):
    if p not in keep:
      p.unlink(missing_ok=True)
      _meta_path(p).unlink(missing_ok=True)
