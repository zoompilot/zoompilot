"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

jetlink's comma side on this fork's tinygrad. jetlink's own tests run it on a
numpy stand-in, since its CI installs neither tinygrad nor openpilot; what
only the real one can show is here: the small model's reset on a real
TinyJit, the warp built from comma's graph and captured under the names every
frame passes, and the large model's frame path from the camera buffer to the
wire.
"""
import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from tinygrad import Tensor

from jetlink.openpilot.warp import prepare_reset

from openpilot.common.test import OpenpilotTestCase

ROOT = Path(__file__).resolve().parents[4]


class TestPrepareReset(OpenpilotTestCase):
  """The small model's history, cleared after the large model drove, on the
  buffers its JIT captured: nothing allocated or compiled on the failure frame."""

  def test_reset_clears_history_in_place_on_repeated_fallbacks(self):
    model = SimpleNamespace(
      input_queues={k: Tensor.zeros(4, 8, device='CPU').contiguous().realize()
                    for k in ('img_q', 'big_img_q', 'feat_q', 'desire_q')},
      prev_desire=np.ones(8), npy={'prev_feat': np.ones(32), 'desire': np.ones(8)})
    identities = {k: id(v) for k, v in model.input_queues.items()}
    reset = prepare_reset(model)
    for _ in range(3):
      for q in model.input_queues.values():
        q.assign(7).realize()
      model.prev_desire.fill(1)
      for v in model.npy.values():
        v.fill(1)
      reset()
      self.assertEqual({k: id(v) for k, v in model.input_queues.items()}, identities)
      for q in model.input_queues.values():
        np.testing.assert_array_equal(q.numpy(), 0)
      np.testing.assert_array_equal(model.prev_desire, 0)
      for v in model.npy.values():
        np.testing.assert_array_equal(v, 0)

  def test_reset_takes_a_modeld_v2_bundle_as_it_is(self):
    # a split bundle: no feature queue, numpy inputs under numpy_inputs, and the
    # packed NPY tensor and the two transforms the warp reads are not queues
    packed = np.ones(16, dtype=np.float32)
    model = SimpleNamespace(
      input_queues={**{k: Tensor.zeros(4, 8, device='CPU').contiguous().realize() for k in ('img_q', 'big_img_q', 'desire_q')},
                    'packed_npy_inputs': Tensor(packed, device='NPY').realize()},
      prev_desire=np.ones(8),
      numpy_inputs={'desire': packed[:8], 'lateral_control_params': packed[8:10], 'prev_desired_curv': packed[10:],
                    'tfm': np.ones((3, 3), dtype=np.float32), 'big_tfm': np.ones((3, 3), dtype=np.float32)})
    reset = prepare_reset(model)
    for q in ('img_q', 'big_img_q', 'desire_q'):
      model.input_queues[q].assign(7).realize()
    reset()
    for q in ('img_q', 'big_img_q', 'desire_q'):
      np.testing.assert_array_equal(model.input_queues[q].numpy(), 0)
    np.testing.assert_array_equal(model.prev_desire, 0)
    np.testing.assert_array_equal(packed, 0)  # every numpy input is a view into it
    np.testing.assert_array_equal(model.input_queues['packed_npy_inputs'].numpy(), 0)

  def test_it_reads_what_openpilots_model_states_have(self):
    # prepare_reset is duck-typed on these names; a sync that renames one
    # breaks every fallback to the small model
    import ast
    for path, npy in ((ROOT / 'openpilot/selfdrive/modeld/modeld.py', 'npy'),
                      (ROOT / 'openpilot/sunnypilot/modeld_v2/modeld.py', 'numpy_inputs')):
      tree = ast.parse(path.read_text())
      cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ModelState')
      assigned = {n.attr for n in ast.walk(cls) if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store)
                  and isinstance(n.value, ast.Name) and n.value.id == 'self'}
      self.assertLessEqual({'input_queues', npy, 'prev_desire'}, assigned, path.name)


# Run as its own process on tinygrad's CPU device: a Metal device on a Mac
# cannot take a camera buffer by pointer (Tensor.from_blob), and the device
# is fixed at the first use in a process
FRAME_PATH = '''
import json, shutil, tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import numpy as np

from openpilot.sunnypilot import jetlink_adapter
from jetlink.openpilot.warp import Warp, Warps, call_warp, compile_warp, init_device
from jetlink.spec import ModelSpec

CAM, MODEL = (1928, 1208), (512, 256)
tmp = Path(tempfile.mkdtemp())
op = jetlink_adapter.adapter()
op.warp_path = lambda *geometry: tmp / 'warp.pkl'

log = mock.MagicMock()
init_device(log)
found = {'init_failed': log.exception.called}

graph, size = op.make_warp(*CAM, *MODEL)
compile_warp(graph, size, op.warp_path())
jit = Warps(op).load(*CAM, *MODEL)
found['names'] = list(jit.captured.expected_names)
face = op.model_face()
found['frame_size'] = size == face.frame_size(*CAM)
warp = Warp(jit, face.frame_size(*CAM), log)

# Cinque Terre V3's inputs and output layout, read off its ONNX
SLICES = {'lane_lines': (0, 528), 'lane_lines_prob': (528, 536), 'road_edges': (536, 800), 'meta': (800, 855),
          'desire_pred': (855, 887), 'pose': (887, 899), 'wide_from_device_euler': (899, 905),
          'road_transform': (905, 917), 'plan': (917, 1907), 'lead': (1907, 2051), 'lead_prob': (2051, 2054),
          'desire_state': (2054, 2062), 'action': (2062, 2066), 'hidden_state': (2066, 18450), 'pad': (18450, 18452)}
INPUTS = {'new_img': (2, 6, 128, 256), 'desire': (8,), 'traffic_convention': (1, 2), 'action_t': (1, 2),
          'state_img_q': (2, 5, 6, 128, 256), 'state_desire_q': (132, 1, 8), 'state_feat_q': (128, 1, 16384)}
outputs = {'outputs': (1, 18452), **{f'next_{n}': s for n, s in INPUTS.items() if n.startswith('state_')}}
spec = ModelSpec(sha256='a' * 64, nbytes=1, frame_skip=4, input_shapes=INPUTS, output_shapes=outputs,
                 output_slices={k: slice(*v) for k, v in SLICES.items()}, checkpoint=None)

class Client:
  def __init__(self):
    self.sent, self.last_timings, self.last_state, self.dead = [], (0, 0, 0), {'gpu_temp': 40.0}, False
    self.last_output, self.unanswered = None, 0
    self.t = SimpleNamespace(link_info=lambda: {'kind': 'usb'})
  def infer_begin(self, data, packed, frame_id, reset=False, want_state=False, skip_if_busy=False):
    self.sent.append((bytes(data), np.array(packed), reset, want_state))
    return frame_id
  def infer_end(self, seq, deadline=None, hold=None):
    self.last_output = np.zeros(18452, np.float32)
    return self.last_output
  def drain(self):
    return 0

from jetlink.openpilot.model_state import JetlinkModelState
client, events = Client(), []
model = JetlinkModelState(client, spec, warp, face=face, log=log, event=lambda name, **f: events.append(name))
rng = np.random.default_rng(1)
frames = {k: rng.integers(0, 256, face.frame_size(*CAM), dtype=np.uint8) for k in ('img', 'big_img')}
bufs = {k: SimpleNamespace(data=v) for k, v in frames.items()}
tfm = {'img': np.eye(3, dtype=np.float32), 'big_img': np.eye(3, dtype=np.float32) * 0.9}
tfm['big_img'][2, 2] = 1.0
out = None
for i in range(3):
  out = model.run(bufs, tfm, {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
                              'action_t': np.array([0.1, 0.2], np.float32)})

# the same frame warped directly: what the model sent is the warp's output
from tinygrad.device import Device
from tinygrad.tensor import Tensor
blobs = {k: Tensor.from_blob(v.ctypes.data, (v.size,), dtype='uint8', device=Device.DEFAULT) for k, v in frames.items()}
direct = call_warp(jit, Tensor(tfm['img'], device='NPY').realize(), Tensor(tfm['big_img'], device='NPY').realize(),
                   blobs['img'], blobs['big_img']).numpy().tobytes()
found.update(device=Device.DEFAULT, sent=len(client.sent), bytes=len(client.sent[0][0]), expected=int(np.prod(spec.warped_shape)),
             same=client.sent[-1][0] == direct, resets=[s[2] for s in client.sent], asks=[s[3] for s in client.sent],
             parsed=sorted(out), events=events)
shutil.rmtree(tmp)
print(json.dumps(found))
'''


class TestTheFramePath(OpenpilotTestCase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    env = {**os.environ, 'DEV': 'CPU', 'PYTHONPATH': os.pathsep.join([str(ROOT), *sys.path])}
    out = subprocess.run([sys.executable, '-c', FRAME_PATH], capture_output=True, text=True, env=env, cwd=str(ROOT),
                         timeout=600)
    assert out.returncode == 0, out.stderr
    cls.found = json.loads(out.stdout.strip().splitlines()[-1])

  def test_the_gpu_comes_up_before_modeld_goes_realtime(self):
    # tinygrad's device and its compile pool, started on the caller's thread
    self.assertFalse(self.found['init_failed'])

  def test_the_warp_is_captured_under_the_names_every_frame_passes(self):
    from jetlink.openpilot.warp import WARP_INPUT_NAMES
    self.assertEqual(self.found['names'], WARP_INPUT_NAMES)
    self.assertTrue(self.found['frame_size'], "make_warp's frame is not the camera buffer modeld hands over")

  def test_the_large_model_sends_the_warped_frame(self):
    found = self.found
    self.assertEqual(found['device'], 'CPU')
    self.assertEqual(found['sent'], 3)
    self.assertEqual(found['bytes'], found['expected'])
    self.assertTrue(found['same'], "the model sent something other than the warp's output")
    self.assertEqual(found['resets'], [True, False, False])

  def test_it_parses_the_reply_with_comma_s_parser(self):
    for key in ('plan', 'lane_lines', 'lead', 'meta', 'desire_state', 'action'):
      self.assertIn(key, self.found['parsed'])

  def test_it_asks_for_telemetry_on_its_own(self):
    # modeld passes no callback: a frame asks only once the 1 Hz log is due,
    # and the first frame's log has just gone out
    self.assertEqual(self.found['asks'], [False, False, False])
    self.assertEqual(self.found['events'], ['jetlinkTelemetry'])
