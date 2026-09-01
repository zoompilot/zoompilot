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
"""
from __future__ import annotations

import os
from collections.abc import Callable

import numpy as np
from tinygrad.tensor import Tensor

from msgq.visionipc import VisionBuf

from openpilot.common.file_chunker import open_file_chunked
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.compile_modeld import WARP_INPUTS, make_warp_input_queues
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.helpers import get_tg_input_devices, load_oob, modeld_pkl_path
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase

PROCESS_NAME = "openpilot.selfdrive.modeld.modeld"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')


class JetlinkModelState(ModelStateBase):
  """Duck-types selfdrive.modeld.modeld.ModelState."""

  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  def __init__(self, cam_w: int, cam_h: int, client, spec):
    ModelStateBase.__init__(self)
    self.client = client
    self.spec = spec
    # Not "chestnut" hardware, but chestnut semantics: this is the big model,
    # and modelV2.big / the UI / the model manager all key off this flag.
    self.chestnut = True

    devices = get_tg_input_devices(PROCESS_NAME, chestnut=False)
    self.WARP_DEV, self.QUEUE_DEV = devices['WARP_DEV'], devices['QUEUE_DEV']

    # The warp JIT depends only on camera and model geometry, not on which
    # model runs afterwards, and the small and large models share a 256x128
    # input. So reuse the small model's warp and never compile a large tinygrad
    # pkl on the comma - which is the expensive step jetlink exists to avoid.
    jits = load_oob(open_file_chunked(modeld_pkl_path(chestnut=False)))
    self.warp = jits[(cam_w, cam_h)]
    small_img = jits['metadata']['input_shapes']['img']
    if tuple(small_img[2:]) != tuple(spec.input_shapes['img'][2:]):
      raise RuntimeError(f"warp geometry {small_img[2:]} does not match the large model "
                         + f"{spec.input_shapes['img'][2:]}; a large-model warp JIT is needed")

    self.input_shapes = spec.input_shapes
    self.output_slices = spec.output_slices
    self.vision_input_names = [k for k in spec.input_shapes if 'img' in k]
    # From the spec, not recomputed from ModelConstants: the server derives its
    # history stride from the same field, and two independent derivations would
    # diverge silently for a model with a different skip.
    self.frame_skip = spec.frame_skip

    self.input_queues, self.npy = make_warp_input_queues(
      spec.input_shapes, self.frame_skip, device=self.QUEUE_DEV)

    # Mirrors compile_modeld.make_input_queues' packed_npy_inputs, minus the GPU
    # queues, which the server owns. Same layout, so the two ends agree.
    self.packed = np.zeros(spec.packed_nelem, dtype=np.float32)
    views = np.split(self.packed, np.cumsum(spec.packed_sizes[:-1]))
    self.npy.update({k: v.reshape(s) for (k, s), v in
                     zip(spec.packed_shapes.items(), views, strict=True)})

    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    self.parser = Parser()
    self.frame_buf_params = {k: get_nv12_info(cam_w, cam_h) for k in ('img', 'big_img')}
    self.full_frames: dict[str, Tensor] = {}
    self._blob_cache: dict[tuple[str, int], Tensor] = {}
    self._need_reset = True
    self._frame_id = 0

  def make_chestnut_state(self, pm):
    from openpilot.sunnypilot.jetlink.state import JetlinkChestnutState
    return JetlinkChestnutState(pm, self.client)

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    return {k: model_outputs[np.newaxis, v] for k, v in output_slices.items()}

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray], after_enqueue: Callable[[], None] | None = None) -> dict[str, np.ndarray]:
    for key in bufs.keys():
      ptr = np.frombuffer(bufs[key].data, dtype=np.uint8).ctypes.data
      yuv_size = self.frame_buf_params[key][3]
      cache_key = (key, ptr)
      if cache_key not in self._blob_cache:
        self._blob_cache[cache_key] = Tensor.from_blob(ptr, (yuv_size,), dtype='uint8', device=self.WARP_DEV)
      self.full_frames[key] = self._blob_cache[cache_key]

    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    inputs['desire_pulse'][0] = 0
    self.npy['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    self.npy['traffic_convention'][:] = inputs['traffic_convention']
    self.npy['action_t'][:] = inputs['action_t']
    self.npy['tfm'][:, :] = transforms['img'][:, :]
    self.npy['big_tfm'][:, :] = transforms['big_img'][:, :]

    warped = self.warp(**{k: self.input_queues[k] for k in WARP_INPUTS},
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
