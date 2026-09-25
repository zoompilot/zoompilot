"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

What the comma puts on the wire for each kind of big model.

A queued graph (up to Cinque Terre V2) gets the frame, the scalars and last
frame's hidden state; a stateful one (openpilot #38916, Cinque Terre V3 on)
keeps its hidden state on the Jetson and gets the frame and the scalars only.
The warp and the link are faked; the packing is the code that drives.
"""
from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from jetlink.spec import ModelSpec
from openpilot.sunnypilot.accelerators.jetlink import model_state

# Cinque Terre V3's output layout, read off its ONNX
SLICES = {'lane_lines': (0, 528), 'lane_lines_prob': (528, 536), 'road_edges': (536, 800), 'meta': (800, 855),
          'desire_pred': (855, 887), 'pose': (887, 899), 'wide_from_device_euler': (899, 905),
          'road_transform': (905, 917), 'plan': (917, 1907), 'lead': (1907, 2051), 'lead_prob': (2051, 2054),
          'desire_state': (2054, 2062), 'action': (2062, 2066), 'hidden_state': (2066, 18450), 'pad': (18450, 18452)}
STATEFUL = {
  'new_img': (2, 6, 128, 256), 'desire': (8,), 'traffic_convention': (1, 2), 'action_t': (1, 2),
  'state_img_q': (2, 5, 6, 128, 256), 'state_desire_q': (132, 1, 8), 'state_feat_q': (128, 1, 16384),
}
QUEUED = {
  'img': (1, 12, 128, 256), 'big_img': (1, 12, 128, 256), 'desire_pulse': (1, 25, 8),
  'traffic_convention': (1, 2), 'action_t': (1, 2), 'features_buffer': (1, 32, 32, 512),
}


def spec_for(inputs: dict) -> ModelSpec:
  outputs = {'outputs': (1, 18452)}
  outputs.update({f'next_{n}': s for n, s in inputs.items() if n.startswith('state_')})
  return ModelSpec(sha256='a' * 64, nbytes=1, frame_skip=4, input_shapes=inputs, output_shapes=outputs,
                   output_slices={k: slice(*v) for k, v in SLICES.items()}, checkpoint=None)


class FakeClient:
  def __init__(self):
    self.sent = []
    self.last_timings = (0, 0, 0)
    self.t = SimpleNamespace()
    self.output = np.zeros(18452, np.float32)
    self.output[slice(*SLICES['hidden_state'])] = 0.5

  def infer_begin(self, data, packed, frame_id, reset=False, want_state=False):
    self.sent.append((np.frombuffer(bytes(data), np.uint8).copy(), np.array(packed, copy=True), frame_id, reset))
    return frame_id

  def infer_end(self, seq):
    return self.output


class TestWire(unittest.TestCase):
  def run_frames(self, inputs: dict, n: int = 3):
    spec = spec_for(inputs)
    client = FakeClient()
    warped = np.arange(np.prod(spec.warped_shape), dtype=np.uint64).astype(np.uint8)
    with mock.patch.object(model_state.warp_cache, 'call_warp', return_value=SimpleNamespace(data=lambda: warped)), \
         mock.patch.object(model_state.Tensor, 'from_blob', return_value=object()):
      state = model_state.JetlinkModelState(1928, 1208, client, spec, warp=object())
      bufs = {k: SimpleNamespace(data=np.zeros(8, np.uint8)) for k in ('img', 'big_img')}
      for i in range(n):
        desire = np.zeros(8, np.float32)
        desire[3] = 1.0 if i >= 1 else 0.0   # held from frame 1: a pulse on 1 only
        state.run(bufs, {'img': np.eye(3), 'big_img': np.eye(3)},
                  {'desire_pulse': desire, 'traffic_convention': np.array([1, 0], np.float32),
                   'action_t': np.array([0.1, 0.2], np.float32)})
    return spec, state, client, warped

  def test_a_stateful_model_gets_the_frame_and_twelve_floats(self):
    spec, state, client, warped = self.run_frames(STATEFUL)
    self.assertEqual(state.vision_input_names, ['img', 'big_img'])
    self.assertNotIn('prev_feat', state.npy)
    for i, (data, packed, frame_id, reset) in enumerate(client.sent):
      self.assertEqual(frame_id, i + 1)
      self.assertEqual(reset, i == 0)
      np.testing.assert_array_equal(data, warped)
      self.assertEqual(packed.shape, (12,))
      np.testing.assert_array_equal(packed[8:], np.array([1, 0, 0.1, 0.2], np.float32))
    # the desire pulse is the rising edge, as openpilot's own ModelState sends it
    self.assertEqual([p[3] for _, p, _, _ in client.sent], [0.0, 1.0, 0.0])

  def test_a_queued_model_still_sends_the_hidden_state_back(self):
    spec, state, client, _ = self.run_frames(QUEUED)
    first, second = client.sent[0][1], client.sent[1][1]
    self.assertEqual(first.shape, (spec.packed_nelem,))
    self.assertTrue((first[-16384:] == 0).all())
    self.assertTrue((second[-16384:] == 0.5).all())

  def test_the_warp_is_sized_from_either_layout(self):
    for inputs in (STATEFUL, QUEUED):
      self.assertEqual(spec_for(inputs).model_hw, (128, 256))


if __name__ == '__main__':
  unittest.main()
