"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

jetlink's comma side on this fork's tinygrad. jetlink's own tests run it on a
numpy stand-in, since its CI installs neither tinygrad nor openpilot; what
only the real one can show is here: the small model's reset on a real
TinyJit, on the buffers each of openpilot's ModelStates keeps, and the large
model's frame path from the camera buffer to the wire through modeld's warp.
"""
import ast
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from tinygrad import Tensor

from jetlink.openpilot.warp import prepare_reset

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

ROOT = Path(__file__).resolve().parents[4]


def state(*shape, dtype=np.float32):
  return Tensor(np.zeros(shape, dtype=dtype)).contiguous().realize()


def assigned(path: Path, cls: str) -> set[str]:
  """Every self.<attr> a class of `path` assigns."""
  tree = ast.parse(path.read_text())
  node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
  return {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store)
          and isinstance(n.value, ast.Name) and n.value.id == 'self'}


class TestPrepareReset(OpenpilotTestCase):
  """The small model's history, cleared after the large model drove, on the
  buffers its JIT captured: nothing allocated or compiled on the failure frame."""

  def test_stock_modelds_state_clears_in_place_on_repeated_fallbacks(self):
    from openpilot.selfdrive.modeld.modeld import input_view
    queues = {'state_img_q': state(5, 6, 2, 4, dtype=np.uint8), 'state_feat_q': state(8, 1, 16)}
    pairs = {name: f'next_{name}' for name in queues}
    # the model writes its next state into the same buffers, through views
    outputs = {pairs[name]: input_view(q._buffer(), q.shape, q.dtype, 0) for name, q in queues.items()}
    packed = np.ones(64, np.uint8)
    model = SimpleNamespace(input_queues={**queues, 'desire': state(8)}, state_pairs=pairs, packed_input=packed,
                            prev_desire=np.ones(8))
    identities = {k: id(v) for k, v in model.input_queues.items()}
    reset = prepare_reset(model)
    for _ in range(3):
      for q in model.input_queues.values():
        q.assign(7).realize()
      packed.fill(1)
      model.prev_desire.fill(1)
      reset()
      self.assertEqual({k: id(v) for k, v in model.input_queues.items()}, identities)
      for name in queues:
        np.testing.assert_array_equal(model.input_queues[name].numpy(), 0)
        np.testing.assert_array_equal(outputs[pairs[name]].numpy(), 0)
      # a view of the packed upload, which the next frame writes before it runs
      np.testing.assert_array_equal(model.input_queues['desire'].numpy(), 7)
      np.testing.assert_array_equal(packed, 0)
      np.testing.assert_array_equal(model.prev_desire, 0)

  def test_a_modeld_v2_native_bundle(self):
    packed = np.ones(64, np.uint8)
    adapter = SimpleNamespace(is_native=True, input_queues={'state_img_q': state(5, 6, 2, 4, dtype=np.uint8)},
                              state_pairs={'state_img_q': 'next_state_img_q'}, packed_input=packed)
    model = SimpleNamespace(adapter=adapter, prev_desire=np.ones(8))
    reset = prepare_reset(model)
    adapter.input_queues['state_img_q'].assign(7).realize()
    reset()
    np.testing.assert_array_equal(adapter.input_queues['state_img_q'].numpy(), 0)
    np.testing.assert_array_equal(packed, 0)
    np.testing.assert_array_equal(model.prev_desire, 0)

  def test_a_modeld_v2_legacy_bundle(self):
    # a split bundle: no feature queue, and the packed NPY tensor is not a
    # queue; every numpy input is a view into it
    packed = np.ones(16, dtype=np.float32)
    queues = {k: state(4, 8) for k in ('img_q', 'big_img_q', 'desire_q')}
    adapter = SimpleNamespace(is_native=False, input_queues={**queues, 'packed_npy_inputs': Tensor(packed, device='NPY').realize()},
                              numpy_inputs={'desire': packed[:8], 'lateral_control_params': packed[8:10], 'prev_desired_curv': packed[10:],
                                            'tfm': np.ones((3, 3), dtype=np.float32), 'big_tfm': np.ones((3, 3), dtype=np.float32)})
    model = SimpleNamespace(adapter=adapter, prev_desire=np.ones(8))
    reset = prepare_reset(model)
    for q in queues.values():
      q.assign(7).realize()
    reset()
    for q in queues.values():
      np.testing.assert_array_equal(q.numpy(), 0)
    np.testing.assert_array_equal(model.prev_desire, 0)
    np.testing.assert_array_equal(packed, 0)
    np.testing.assert_array_equal(adapter.input_queues['packed_npy_inputs'].numpy(), 0)

  def test_it_reads_what_openpilots_model_states_have(self):
    # prepare_reset reads these names; a sync that renames one breaks every
    # fallback to the small model
    adapters = ROOT / 'openpilot/sunnypilot/modeld_v2/model_adapters.py'
    for path, cls, names in ((ROOT / 'openpilot/selfdrive/modeld/modeld.py', 'ModelState',
                              {'input_queues', 'state_pairs', 'packed_input', 'prev_desire'}),
                             (ROOT / 'openpilot/sunnypilot/modeld_v2/modeld.py', 'ModelState', {'adapter', 'prev_desire'}),
                             (adapters, 'BaseModelAdapter', {'is_native'}),
                             (adapters, 'NativeTinygradAdapter', {'input_queues', 'state_pairs', 'packed_input', 'is_native'}),
                             (adapters, 'LegacyModelAdapter', {'input_queues', 'numpy_inputs'})):
      with self.subTest(cls, path=path.name):
        self.assertLessEqual(names, assigned(path, cls))


# The warp as the fork's build compiles it (selfdrive/modeld/SConscript), on
# tinygrad's CPU device, with this checkout's tinygrad
CAM, MODEL = (1928, 1208), (512, 256)

# Run as its own process: the device is fixed at the first use in a process
FRAME_PATH = '''
import json, sys
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import numpy as np

from openpilot.sunnypilot import jetlink_adapter
from jetlink.openpilot.warp import Warp, Warps, init_device
from jetlink.spec import ModelSpec

CAM, MODEL = (1928, 1208), (512, 256)
op = jetlink_adapter.adapter()
op.warp_path = lambda *geometry: Path(sys.argv[1])

log = mock.MagicMock()
init_device(log)
found = {'init_failed': log.exception.called}

face = op.model_face()
loaded = Warps(op).load(*CAM, *MODEL)
reads = loaded['input_specs']['input_frame'][0][1]
warp = Warp(loaded, face.frame_size(*CAM))

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
tfm = {'img': np.eye(3, dtype=np.float32), 'big_img': np.eye(3, dtype=np.float32) * 0.9}
tfm['big_img'][2, 2] = 1.0
out, same = None, []
from tinygrad.tensor import Tensor
for i in range(3):
  # camera buffers, a new pair each frame as camerad cycles them
  frames = {k: rng.integers(0, 256, face.frame_size(*CAM), dtype=np.uint8) for k in ('img', 'big_img')}
  bufs = {k: SimpleNamespace(data=v) for k, v in frames.items()}
  out = model.run(bufs, tfm, {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
                              'action_t': np.array([0.1, 0.2], np.float32)})
  # the same frames through the warp directly, as modeld calls it
  direct = loaded['run'](input_frame=Tensor(np.stack([frames['img'][:reads], frames['big_img'][:reads]])),
                         M_inv=Tensor(np.stack([tfm['img'], tfm['big_img']]))).numpy().tobytes()
  same.append(client.sent[-1][0] == direct)

from tinygrad.device import Device
found.update(device=Device.DEFAULT, reads=reads, sent=len(client.sent), bytes=len(client.sent[0][0]),
             expected=int(np.prod(spec.warped_shape)), same=same, blank=all(not any(s[0]) for s in client.sent),
             resets=[s[2] for s in client.sent], asks=[s[3] for s in client.sent], parsed=sorted(out), events=events)
print(json.dumps(found))
'''


class TestTheFramePath(OpenpilotTestCase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    env = {**os.environ, 'DEV': 'CPU', 'PYTHONPATH': os.pathsep.join([str(ROOT / 'tinygrad_repo'), str(ROOT), *sys.path])}
    with tempfile.TemporaryDirectory() as tmp:
      pkl = Path(tmp) / 'warp.pkl'
      stride, y_height, uv_height, _ = get_nv12_info(*CAM)
      frame = f'{CAM[0]},{CAM[1]},{stride},{y_height},{uv_height},{stride * (y_height + uv_height)}'
      compile_warp = ROOT / 'tinygrad_repo/examples/openpilot/compile_warp.py'
      built = subprocess.run([sys.executable, str(compile_warp), '--frame', frame, '--warp-to', f'{MODEL[0]}x{MODEL[1]}',
                              '--layout', 'yuv420', '--frames', '2', '--output', str(pkl), '--benchmark-runs', '1'],
                             capture_output=True, text=True, env=env, cwd=tmp, timeout=600)
      assert built.returncode == 0, built.stderr
      out = subprocess.run([sys.executable, '-c', FRAME_PATH, str(pkl)], capture_output=True, text=True, env=env, cwd=str(ROOT),
                           timeout=600)
    assert out.returncode == 0, out.stderr
    cls.found = json.loads(out.stdout.strip().splitlines()[-1])

  def test_the_gpu_comes_up_before_modeld_goes_realtime(self):
    # tinygrad's device and its compile pool, started on the caller's thread
    self.assertFalse(self.found['init_failed'])

  def test_the_warp_reads_what_modeld_copies_of_a_camera_buffer(self):
    stride, y_height, uv_height, _ = get_nv12_info(*CAM)
    self.assertEqual(self.found['reads'], stride * (y_height + uv_height))

  def test_the_large_model_sends_the_warped_frame(self):
    found = self.found
    self.assertEqual(found['device'], 'CPU')
    self.assertEqual(found['sent'], 3)
    self.assertEqual(found['bytes'], found['expected'])
    self.assertEqual(found['same'], [True] * 3, "the model sent something other than the warp's output")
    self.assertFalse(found['blank'])
    self.assertEqual(found['resets'], [True, False, False])

  def test_it_parses_the_reply_with_comma_s_parser(self):
    for key in ('plan', 'lane_lines', 'lead', 'meta', 'desire_state', 'action'):
      self.assertIn(key, self.found['parsed'])

  def test_it_asks_for_telemetry_on_its_own(self):
    # modeld passes no callback: a frame asks only once the 1 Hz log is due,
    # and the first frame's log has just gone out
    self.assertEqual(self.found['asks'], [False, False, False])
    self.assertEqual(self.found['events'], ['jetlinkTelemetry'])
