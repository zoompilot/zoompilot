from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld.helpers import check_camera_jit, check_modeld_pkl


class TestCheckModeldPkl(OpenpilotTestCase):
  def test_pre_run_model_schema_is_named(self):
    # the pkl shape compile_modeld.py wrote before the warp and policy jits were fused:
    # sunnypilot staging 3e87c0f shipped one against a modeld that reads the fused shape
    stale = {'metadata': {}, 'run_policy': None, (1928, 1208): None, (1344, 760): None}
    with self.assertRaisesRegex(RuntimeError, r"driving_tinygrad.pkl is missing .*input_devices.*run_model"):
      check_modeld_pkl(stale, "driving_tinygrad.pkl")


class TestCheckCameraJit(OpenpilotTestCase):
  # a prebuilt cut on a comma four carries only its own 1344x760 driver camera, so a 3X
  # installing it has no jit for 1928x1208. Name the camera instead of raising a bare KeyError
  def test_camera_the_build_lacks_is_named(self):
    mici_only = {'metadata': {}, 'run_policy': None, (1344, 760): None}
    with self.assertRaisesRegex(RuntimeError, r"1928x1208 driver camera.*has only \[1344x760\]"):
      check_camera_jit(mici_only, 1928, 1208, "driving_tinygrad.pkl")

  def test_camera_the_build_has_passes(self):
    both = {'metadata': {}, (1344, 760): None, (1928, 1208): None}
    for cam_w, cam_h in ((1344, 760), (1928, 1208)):
      check_camera_jit(both, cam_w, cam_h, "driving_tinygrad.pkl")
