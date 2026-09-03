"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A ModelState whose policy runs on the Jetson.

Everything that touches the car stays on the comma: cameras, calibration, the
warp, the parser, controlsd, panda, CAN. The Jetson is a pure function - warped
frames and context in, 18452 floats out - so it holds no control state and
never sees the bus.

The split is at `run_policy`. `warp` still runs on the comma's own GPU, because
its input is a 2 MB camera buffer already in GPU memory and its output is the
393 KB the link has to carry anyway. The history queues (img_q, big_img_q,
feat_q, desire_q) live on the Jetson: shipping them per frame would cost ~10 MB
instead of ~0.5 MB.

Upstream fused warp and policy into one JIT, so there is no longer a warp in
the small model's pkl to borrow. jetlinkd compiles a standalone one offroad;
see warp_cache. The wire format is unchanged - the same `Tensor.cat` of the two
warped frames goes out - because `make_warp` is still the same closure the
fused JIT wraps.
"""
from __future__ import annotations

import os
from collections.abc import Callable

import numpy as np
from tinygrad.device import Device
from tinygrad.tensor import Tensor

from msgq.visionipc import VisionBuf

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.sunnypilot.accelerators.jetlink import warp_cache
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase

SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')


class JetlinkModelState(ModelStateBase):
  """Duck-types selfdrive.modeld.modeld.ModelState."""

  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  def __init__(self, cam_w: int, cam_h: int, client, spec, small=None):
    ModelStateBase.__init__(self)
    self.client = client
    self.spec = spec
    # Not chestnut hardware, but the same role: modelV2.big, the UI and the
    # model manager all key off this flag.
    self.chestnut = True

    # The warp depends only on camera and model geometry, not on which model
    # runs afterwards, and the small and large models share a 256x128 input.
    # Upstream used to ship it as its own JIT inside the small model's pkl and
    # this borrowed it; now the pkl carries one fused warp+policy graph with no
    # seam, so jetlinkd compiles a standalone warp offroad and this loads it.
    # Either way the comma never compiles a large tinygrad pkl, which is the
    # expensive step jetlink exists to avoid.
    # make_warp is sized in NV12 pixels, the model input in post-deinterleave
    # ones: frames_to_tensor halves both axes turning (model_h*3//2, model_w)
    # YUV into (6, model_h//2, model_w//2). So img (1, 12, 128, 256) is a warp
    # of 512x256, which is MEDMODEL_INPUT_SIZE, which is what jetlinkd built.
    img_h, img_w = spec.input_shapes['img'][2:]
    self.warp = warp_cache.load_warp(cam_w, cam_h, img_w * 2, img_h * 2)

    # modeld still hands us its small ModelState, now only to check that the
    # geometry the warp was built for is the geometry the large model wants.
    small_img = small.input_shapes['img'] if small is not None else None
    if small_img is not None and tuple(small_img[2:]) != tuple(spec.input_shapes['img'][2:]):
      raise RuntimeError(f"warp geometry {small_img[2:]} does not match the large model "
                         + f"{spec.input_shapes['img'][2:]}; a large-model warp JIT is needed")

    self.input_shapes = spec.input_shapes
    self.output_slices = spec.output_slices
    self.vision_input_names = [k for k in spec.input_shapes if 'img' in k]
    # From the spec, not recomputed from ModelConstants: the server derives its
    # history stride from the same field, and two independent derivations would
    # diverge silently for a model with a different skip.
    self.frame_skip = spec.frame_skip

    # The warp's own two inputs. Upstream keeps these inside its packed buffer;
    # here they stay separate NPY tensors, because everything downstream of the
    # warp is the server's and never reads this buffer. An NPY tensor wraps the
    # numpy array it was built from, so writing the array is what feeds the JIT.
    self.npy = {'tfm': np.zeros((3, 3), dtype=np.float32),
                'big_tfm': np.zeros((3, 3), dtype=np.float32)}
    self.warp_inputs = {k: Tensor(v, device='NPY').realize() for k, v in self.npy.items()}

    # Mirrors compile_modeld.make_input_queues' packed_npy_inputs, minus the GPU
    # queues, which the server owns. Same layout, so the two ends agree.
    self.packed = np.zeros(spec.packed_nelem, dtype=np.float32)
    views = np.split(self.packed, np.cumsum(spec.packed_sizes[:-1]))
    self.npy.update({k: v.reshape(s) for (k, s), v in
                     zip(spec.packed_shapes.items(), views, strict=True)})

    # Upstream dropped its WARP_DEV/QUEUE_DEV split when it fused the graph, so
    # the warp runs on the default device. Read it once here rather than per
    # frame: it has to be the device jetlinkd compiled the cached JIT against.
    self.warp_dev = Device.DEFAULT
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    self.parser = Parser()
    self.frame_buf_params = {k: get_nv12_info(cam_w, cam_h) for k in ('img', 'big_img')}
    self.full_frames: dict[str, Tensor] = {}
    self._blob_cache: dict[tuple[str, int], Tensor] = {}
    self._need_reset = True
    self._frame_id = 0

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    return {k: model_outputs[np.newaxis, v] for k, v in output_slices.items()}

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray], after_enqueue: Callable[[], None] | None = None) -> dict[str, np.ndarray]:
    for key in bufs.keys():
      ptr = np.frombuffer(bufs[key].data, dtype=np.uint8).ctypes.data
      yuv_size = self.frame_buf_params[key][3]
      cache_key = (key, ptr)
      if cache_key not in self._blob_cache:
        self._blob_cache[cache_key] = Tensor.from_blob(ptr, (yuv_size,), dtype='uint8', device=self.warp_dev)
      self.full_frames[key] = self._blob_cache[cache_key]

    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    inputs['desire_pulse'][0] = 0
    self.npy['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    self.npy['traffic_convention'][:] = inputs['traffic_convention']
    self.npy['action_t'][:] = inputs['action_t']
    self.npy['tfm'][:, :] = transforms['img'][:, :]
    self.npy['big_tfm'][:, :] = transforms['big_img'][:, :]

    warped = self.warp(**self.warp_inputs,
                       frame=self.full_frames['img'], big_frame=self.full_frames['big_img'])

    self._frame_id += 1
    # .data() rather than .numpy(): same mean cost (~4.4 ms, this is a
    # write-combined GPU mapping read at ~90 MB/s and unavoidable), but it
    # drops a per-frame allocation and, measured on the car, a 52 ms outlier
    # that .numpy() produces. On a 50 ms budget the tail is what matters.
    # The memoryview goes straight to the wire with no numpy round trip.
    seq = self.client.infer_begin(warped.data(), self.packed, self._frame_id,
                                  reset=self._need_reset, want_state=after_enqueue is not None)
    self._need_reset = False
    # Publish health while the Jetson works, exactly where modeld puts it.
    if after_enqueue is not None:
      after_enqueue()
    # Blocks like a chestnut frame does. A long frame is a dropped camera
    # frame, which modeld counts; only a stall past the client's FRAME_TIMEOUT
    # raises, and that lands in modeld's fallback to the small model.
    model_output = self.client.infer_end(seq)

    # The non-finite guard that upstream's ModelState.run does here runs on the
    # server instead (session.on_infer), which reports Status.NOT_FINITE; the
    # client turns that into a LinkError, so modeld's big->small failover fires
    # exactly as it does for a chestnut. Checking again here would rescan 18452
    # floats on the comma for nothing.
    outputs_dict = self.parser.parse_outputs(self.slice_outputs(model_output, self.output_slices))
    self.npy['prev_feat'][:] = model_output[self.output_slices['hidden_state']]
    if SEND_RAW_PRED:
      outputs_dict['raw_pred'] = model_output.copy()
    return outputs_dict

  def close(self) -> None:
    """Let go of the link. modeld calls this on a big model that finished
    loading after it had already given up and started the small one."""
    self.client.close()

  def warmup(self) -> None:
    dummy_frames = {k: np.zeros(self.frame_buf_params[k][3], dtype=np.uint8) for k in self.vision_input_names}
    eye = np.eye(3, dtype=np.float32)
    dims = {'desire_pulse': ModelConstants.DESIRE_LEN, 'traffic_convention': 2, 'action_t': 2}
    self.run(dummy_frames, dict.fromkeys(self.vision_input_names, eye),
             {k: np.zeros(v, dtype=np.float32) for k, v in dims.items()})
    # Drop the warm-up frame from both ends' history.
    self.packed[:] = 0
    self.prev_desire[:] = 0
    self.full_frames.clear()
    self._blob_cache.clear()
    self._need_reset = True
    cloudlog.warning("jetlink: warmup complete")
