#!/usr/bin/env python3
import unittest

from opendbc.car import DT_CTRL
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaFlags, MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.mazda.values import MazdaSafetyFlagsSP
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  """Pre-2022 EPS: upstream's envelope, no safety param bit."""

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  # camera 0x243/0x440 frames forward while openpilot is not controlling
  FWD_BLACKLISTED_ADDRS = {2: []}

  SAFETY_PARAM = 0

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.SAFETY_PARAM)
    self.safety.init_tests()

  @classmethod
  def controller_params(cls):
    # the CarControllerParams branch this envelope pairs with: values.py keys it on the same
    # EPS bit interface.py hands the panda
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5
      flags = MazdaFlags.STEER_TO_ZERO_EPS if cls.SAFETY_PARAM & MazdaSafetyFlags.STEER_TO_ZERO_EPS else 0
    return CarControllerParams(FakeCP())

  def test_controller_rate_limits_equal_the_pandas(self):
    # driver_limit_check demands a retreat of at least max_rate_down per frame once the driver
    # bound is below the last command, and rejects anything above max_rate_up on the way up,
    # so the controller's per-frame deltas must be exactly the panda's, not merely within them
    params = self.controller_params()
    self.assertEqual(params.STEER_DELTA_UP, self.MAX_RATE_UP)
    self.assertEqual(params.STEER_DELTA_DOWN, self.MAX_RATE_DOWN)
    self.assertEqual(params.STEER_MAX, self.MAX_TORQUE)
    self.assertEqual(params.STEER_DRIVER_ALLOWANCE, self.DRIVER_TORQUE_ALLOWANCE)
    self.assertEqual(params.STEER_DRIVER_MULTIPLIER, self.DRIVER_TORQUE_FACTOR)

  def _closed_loop(self, params, frames, ctrl_last=0, frame0=0):
    """Run the real controller limiter frame by frame through the compiled safety model.
    frames yields (driver_torque, target); returns (rejected frames, full retreats, last cmd)."""
    rejected, full_retreats = [], 0
    for frame, (driver_torque, target) in enumerate(frames, start=frame0 + 1):
      self.safety.set_timer(frame * 10_000)
      self._rx(self._torque_driver_msg(driver_torque))
      cmd = apply_driver_steer_torque_limits(target, ctrl_last, driver_torque, params, self.MAX_TORQUE)
      if not self._tx(self._torque_cmd_msg(cmd)):
        rejected.append((frame, cmd, ctrl_last, driver_torque))
      full_retreats += abs(cmd) == abs(ctrl_last) - params.STEER_DELTA_DOWN
      ctrl_last = cmd
    return rejected, full_retreats, ctrl_last

  def test_driver_override_winddown_is_never_rejected(self):
    # Closed loop: the command is held at full torque while the driver ramps against it at
    # several slopes; the driver bound then falls faster than the controller retreats, so the
    # panda's max_rate_down requirement binds. Route 00000148 lost 171 consecutive frames here
    # when the panda demanded 25 and the controller retreated 12.
    params = self.controller_params()
    max_torque = self.MAX_TORQUE
    for sign in (1, -1):
      for slope in (1, 2, 5, 10, 30):
        with self.subTest(sign=sign, slope=slope):
          self.safety.init_tests()
          self.safety.set_controls_allowed(True)
          self._reset_torque_driver_measurement(0)
          # ramp up to full torque with no driver input
          ramp = [(0, max_torque * sign)] * (max_torque // self.MAX_RATE_UP + 5)
          rejected, _, last = self._closed_loop(params, ramp)
          self.assertEqual(rejected, [])
          self.assertEqual(last, max_torque * sign)
          # driver pushes back, harder each frame, to the top of the 8-bit sensor field
          rejected, full_retreats, _ = self._closed_loop(
            params, [(-sign * min(slope * f, 127), max_torque * sign) for f in range(300)],
            ctrl_last=last, frame0=len(ramp))
          self.assertEqual(rejected, [], f"{len(rejected)} frames rejected, first {rejected[:1]}")
          # the scenario only proves something if the bound outran the retreat at least once
          if slope * self.DRIVER_TORQUE_FACTOR > self.MAX_RATE_DOWN:
            self.assertGreater(full_retreats, 0)

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _laneinfo_msg(self):
    values = {"LINE_VISIBLE": 0}
    return self.packer.make_can_msg_safety("CAM_LANEINFO", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False, set_m=False, set_p=False, tja=False):
    values = {
      "TJA_BUTTON": tja,
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
      "SET_M": set_m,
      "SET_M_INV": (set_m + 1) % 2,
      "SET_P": set_p,
      "SET_P_INV": (set_p + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))

  def test_steer_safety_check(self):
    # the common test, except that disengaged the camera owns 0x243 (test_stock_passthrough),
    # so the zero-torque frame upstream's rule lets through is vetoed too
    for speed in self._torque_speed_range:
      self._reset_speed_measurement(speed)
      max_torque = self._get_max_torque(speed)
      for enabled in [0, 1]:
        for t in range(int(-max_torque * 1.5), int(max_torque * 1.5)):
          self.safety.set_controls_allowed(enabled)
          self._set_prev_torque(t)
          self.assertEqual(bool(enabled) and abs(t) <= max_torque, self._tx(self._torque_cmd_msg(t)))

  def test_stock_passthrough(self):
    # one sender per address, the Tesla test_stock_lkas_passthrough shape: the camera owns
    # 0x243/0x440 only while openpilot controls neither axis (stock lane keep and dash LDW
    # stay live); engaging either axis hands them to openpilot. The fwd hook forwards the
    # camera copy exactly while the tx hook vetoes ours, and vice versa
    for stock_active, controls_allowed, controls_allowed_lateral in [(True, False, False), (False, True, False), (False, False, True)]:
      self.safety.set_controls_allowed(controls_allowed)
      self.safety.set_controls_allowed_lateral(controls_allowed_lateral)
      for addr, msg in ((0x243, self._torque_cmd_msg(0)), (0x440, self._laneinfo_msg())):
        fwd_bus = 0 if stock_active else -1
        self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(2, addr), f"{addr=:#x} {stock_active=}")
        self.assertEqual(not stock_active, self._tx(msg), f"openpilot tx {addr=:#x} {stock_active=}")


class TestMazdaSteerToZeroEpsSafety(TestMazdaSafety):
  """2022+ steer-to-zero EPS (CX-5 2022, CX-9 2021, EPS swaps): MazdaSafetyFlags.STEER_TO_ZERO_EPS
  selects the 1200-count envelope with the EPS's own 12/12 slew."""

  SAFETY_PARAM = MazdaSafetyFlags.STEER_TO_ZERO_EPS

  MAX_RATE_UP = 12
  MAX_RATE_DOWN = 12
  MAX_TORQUE_LOOKUP = [0], [1200]

  MAX_RT_DELTA = 384

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 15

  def test_legacy_envelope_stays_upstreams_without_the_bit(self):
    # the bit only ever loosens the envelope for the EPS that can take it; with it clear the
    # legacy limits refuse the very first frame above upstream's 800 and the 12-count ramp
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(0)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(TestMazdaSafety.MAX_RATE_UP + 1)))
    self._set_prev_torque(800)
    self.assertFalse(self._tx(self._torque_cmd_msg(801)))


class TestMazdaLongitudinalSafety(TestMazdaSteerToZeroEpsSafety, common.LongitudinalAccelSafetyTest):
  """openpilot longitudinal is only offered on steer-to-zero EPS platforms, so LONG always
  travels with that bit."""

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0], [0x21b, 0], [0x21c, 0], [0x499, 0],
             [0x361, 0], [0x362, 0], [0x363, 0], [0x364, 0], [0x365, 0], [0x366, 0], [0x764, 0],
             [0x21b, 2], [0x21c, 2], [0x499, 2], [0x361, 2], [0x362, 2], [0x363, 2], [0x364, 2], [0x365, 2], [0x366, 2]]

  SAFETY_PARAM = MazdaSafetyFlags.LONG | MazdaSafetyFlags.STEER_TO_ZERO_EPS

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable, "BRAKE_ON": 0}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _accel_msg(self, accel: float, bus: int = 0, active: bool = False):
    values = {"ACCEL_CMD": accel, "ACC_ACTIVE": active}
    return self.packer.make_can_msg_safety("CRZ_INFO", bus, values)

  def _crz_ctrl_cmd_msg(self, active: bool, bus: int = 0):
    values = {"CRZ_ACTIVE": active}
    return self.packer.make_can_msg_safety("CRZ_CTRL", bus, values)

  def _press_set(self):
    # arm the driver-intent qualifier the way every logged engagement does: a wheel press
    # lands 30-70 ms before PEDALS.ACC_ACTIVE rises
    self._rx(self._button_msg(set_m=True))

  def test_enable_control_allowed_from_cruise(self):
    # the common test plus the driver-intent qualifier this mode requires
    self._press_set()
    super().test_enable_control_allowed_from_cruise()

  def test_cruise_without_button_never_arms(self):
    # PEDALS.ACC_ACTIVE alone is the body answering our own fabricated frames; without a
    # SET/RES press heard from the wheel it must not arm controls
    self._rx(self._pcm_status_msg(False))
    for _ in range(12):
      self._rx(self._pcm_status_msg(True))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_button_window_expires(self):
    self._press_set()
    # 10 Hz CRZ_BTNS: run the countdown past the 1 s window with idle button frames
    for _ in range(12):
      self._rx(self._button_msg())
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_armed_controls_latch_past_the_window(self):
    self._press_set()
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    # the window expiring must not drop an active engagement
    for _ in range(12):
      self._rx(self._button_msg())
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_each_engage_button_arms(self):
    for btn in ("set_m", "set_p", "resume"):
      self._rx(self._button_msg(**{btn: True}))
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed(), btn)
      self._rx(self._pcm_status_msg(False))

  def test_cancel_button_exits_controls(self):
    self._press_set()
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    # the driver's cancel press always exits controls
    self._rx(self._button_msg(cancel=True))
    self.assertFalse(self.safety.get_controls_allowed())
    # ACC_ACTIVE alone does not re-arm without a fresh button press
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_camera_bus_accel_actuation_limits(self):
    # the synthetic radar frames are duplicated onto the camera bus; same limits apply there
    for accel in (self.MIN_ACCEL - 1, self.MIN_ACCEL, self.INACTIVE_ACCEL, self.MAX_ACCEL, self.MAX_ACCEL + 1):
      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)
        should_tx = controls_allowed and self.MIN_ACCEL <= accel <= self.MAX_ACCEL
        should_tx = should_tx or accel == self.INACTIVE_ACCEL
        self.assertEqual(should_tx, self._tx(self._accel_msg(accel, bus=2)))

  def test_stock_crz_info_standby_allowed(self):
    # every not-controlling stock pattern pegs the command field high: main-off standby and
    # both armed-idle variants (ACC_SET_ALLOWED follows the brake). All must pass byte-exactly,
    # checksum included, instead of being decoded as a huge accel command.
    def pegged_frame(d4, d5, counter):
      dat = bytes([0x01, 0xff, 0xe3, 0xff, d4, d5, counter])
      return dat + bytes([(0xff - sum(dat)) & 0xff])

    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        for d4, d5 in ((0xc0, 0x00), (0xc0, 0x80), (0xc4, 0x80)):
          for counter in range(16):
            self.assertTrue(self._tx(common.make_msg(bus, 0x21b, 8, pegged_frame(d4, d5, counter))))

        bad_checksum = bytes.fromhex("01ffe3ffc0000000")
        self.assertFalse(self._tx(common.make_msg(bus, 0x21b, 8, bad_checksum)))
        # a pegged frame claiming ACC_ACTIVE must never ride the standby allowance
        self.assertFalse(self._tx(common.make_msg(bus, 0x21b, 8, pegged_frame(0xc6, 0x80, 0x00))))
        # and pegged with stop bits set is not a stock pattern either
        self.assertFalse(self._tx(common.make_msg(bus, 0x21b, 8, pegged_frame(0xc0, 0x84, 0x00))))

  def test_empty_radar_tracks_allowed(self):
    radar_messages = {
      0x499: bytes.fromhex("0008c00000000000"),
      0x361: bytes.fromhex("fff7fefe1fc00080"),
      0x362: bytes.fromhex("fff7fefe1fc78c80"),
      0x363: bytes.fromhex("fff7fefe1fc00000"),
      0x364: bytes.fromhex("fff7fefe1fc00000"),
      0x365: bytes.fromhex("fff7fe7ffbff3fc0"),
      0x366: bytes.fromhex("fff7fe7ffbff3fc0"),
    }

    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        for addr, dat in radar_messages.items():
          self.assertTrue(self._tx(common.make_msg(bus, addr, 8, dat)))

  def test_synthetic_lead_radar_track_allowed_disengaged(self):
    # Permit the measurement fields while requiring the occupied-track template. The slot is
    # perception, not actuation, so it remains valid with controls_allowed low like stock radar
    # reports objects with cruise off.
    lead_frames = [
      "0a4e00001c000000",  # stopped lead at 10.25 m
      "229e00007c00000e",  # lead at 34.56 m, closing slowly
      "22de00ff7c000004",  # lead at 34.81 m, opening slowly
      "000e00001c000000",  # zero range, zero relv corner (the template itself)
      "fffe00fffc00000f",  # max range, max relv corner
    ]
    for bus in (0, 2):
      for hexdat in lead_frames:
        dat = bytes.fromhex(hexdat)
        for controls_allowed in (False, True):
          self.safety.set_controls_allowed(controls_allowed)
          self.assertTrue(self._tx(common.make_msg(bus, 0x364, 8, dat)))

  def test_malformed_lead_radar_track_blocked(self):
    # each corrupts one template-owned field of a valid lead frame
    bad_frames = [
      "229100007c00000e",  # data[1] low nibble off the template
      "229e01007c00000e",  # data[2] not zero
      "229e00007d00000e",  # data[4] template bits wrong
      "229e00007cc0000e",  # data[5] wrong -- the retired capture's empty-slot signature
      "229e00007c00010e",  # data[6] not zero
      "229e00007c00100e",  # data[7] high nibble not zero
    ]
    self.safety.set_controls_allowed(True)
    for bus in (0, 2):
      for hexdat in bad_frames:
        self.assertFalse(self._tx(common.make_msg(bus, 0x364, 8, bytes.fromhex(hexdat))))

  def test_unexpected_radar_tracks_blocked(self):
    bad_messages = {
      0x499: bytes.fromhex("0008c00100000000"),
      0x361: bytes.fromhex("fff7fefe1fc00180"),
      0x362: bytes.fromhex("fff7fefe1fc00080"),
      0x363: bytes.fromhex("fff7fefe1fc00080"),
      0x364: bytes.fromhex("fff7fefe1fc00080"),
      0x365: bytes.fromhex("fff7fe7ffbff3f80"),
      0x366: bytes.fromhex("fff7fe7ffbff3f80"),
    }

    self.safety.set_controls_allowed(True)
    for bus in (0, 2):
      for addr, dat in bad_messages.items():
        self.assertFalse(self._tx(common.make_msg(bus, addr, 8, dat)))

  def test_radar_uds_allowlist(self):
    # tester present and session control only, main bus only
    self.assertTrue(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("023e800000000000"))))
    self.assertTrue(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0210020000000000"))))
    self.assertFalse(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0210030000000000"))))
    self.assertFalse(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0227010000000000"))))
    self.assertFalse(self._tx(common.make_msg(2, 0x764, 8, bytes.fromhex("023e800000000000"))))

  def test_crz_ctrl_active_gated_on_controls(self):
    for bus in (0, 2):
      self.safety.set_controls_allowed(False)
      self.assertFalse(self._tx(self._crz_ctrl_cmd_msg(True, bus)))
      self.assertTrue(self._tx(self._crz_ctrl_cmd_msg(False, bus)))

      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._crz_ctrl_cmd_msg(True, bus)))

  # a stock armed-idle CRZ_INFO standby frame, checksum-correct: what the controller emits
  # from the moment the radar teardown lands
  SYNTHETIC_CRZ_INFO_STANDBY = bytes.fromhex("01ffe3ffc000005d")

  def _acc_armed_msg(self, armed):
    # PEDALS with MRCC armed-but-idle (ACC_OFF), the state that persists across ignition
    values = {"ACC_OFF": armed, "BRAKE_ON": 0}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def test_acc_main_waits_for_the_radar_mastery_latch(self):
    # MADS uses acc_main_on's rising edge while software waits for stock-radar silence. panda
    # cannot receive stock CRZ_INFO because it goes stale at teardown, so it
    # mirrors the latch off the observable stand-in: our own first synthetic CRZ_INFO tx
    # (= the teardown landing) plus 1 s of the 50 Hz PEDALS clock. Both machines then arm on
    # the same frame; before that, MRCC-armed PEDALS must not raise acc_main_on, or the edge
    # is consumed at boot and the software's later MADS window transmits into rejections
    # that starve the EPS of 0x243.
    self.safety.set_mads_params(True, False, False)
    # boot: teardown not landed yet, MRCC main armed from the first frame
    for _ in range(120):
      self._rx(self._acc_armed_msg(True))
      self.assertFalse(self.safety.get_acc_main_on())
      self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._torque_cmd_msg(5)))
    # the teardown lands: the controller starts replaying the radar
    self.assertTrue(self._tx(common.make_msg(0, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
    # the latch completes after 1 s of the 50 Hz PEDALS clock
    for _ in range(50):
      self.assertFalse(self.safety.get_acc_main_on())
      self._rx(self._acc_armed_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._torque_cmd_msg(5)))

  def test_software_guard_is_derived_from_the_pandas(self):
    # values.py derives STOCK_RADAR_GUARD_T so the software's MADS edge trails this file's
    # latch; the derivation's panda term has to be this file's constant, not a copy of it
    import os
    import re
    import opendbc.safety
    header = open(os.path.join(os.path.dirname(opendbc.safety.__file__), "modes", "mazda.h")).read()
    silent_frames = int(re.search(r"#define MAZDA_RADAR_SILENT_FRAMES\s+(\d+)U", header).group(1))
    self.assertEqual(CarControllerParams.PANDA_RADAR_SILENT_T, silent_frames / 50.)  # PEDALS is 50 Hz
    self.assertGreater(CarControllerParams.STOCK_RADAR_GUARD_MARGIN_T, 0.)
    self.assertEqual(CarControllerParams.STOCK_RADAR_GUARD_T,
                     CarControllerParams.STOCK_RADAR_ALIVE_T + CarControllerParams.LONG_STEP * DT_CTRL +
                     CarControllerParams.PANDA_RADAR_SILENT_T + CarControllerParams.STOCK_RADAR_GUARD_MARGIN_T)

  def test_panda_arms_lateral_before_the_carstate_guard_lifts(self):
    # Both MADS machines arm off their own radar-silence guard, and the software's must complete
    # strictly AFTER the panda's: if the software arms first the controller ramps torque from
    # zero at STEER_DELTA_UP per frame into a panda that still rejects every 0x243, and when the
    # panda then arms, its rate limiter (desired_torque_last = 0) allows one step, rejects the
    # 36-84 counts by then commanded, resets, and keeps rejecting until the command falls back
    # under a step -- the 0x243 starvation that latched the camera fault on routes 116/117.
    # Same PEDALS frames to both machines; the stock CRZ_INFO only the software sees; our first
    # synthetic CRZ_INFO tx at the controller's latest possible frame (alive window + LONG_STEP).
    from opendbc.can import CANPacker
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.mazda import mazdacan
    from opendbc.car.mazda.carstate import STOCK_RADAR_ALIVE_FRAMES, STOCK_RADAR_GUARD_FRAMES
    from opendbc.car.mazda.interface import CarInterface
    self.safety.set_mads_params(True, False, False)
    CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha_long=True, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha_long=True,
                                       is_release_sp=False, docs=False)
    CI = CarInterface(CP, CP_SP)
    packer = CANPacker("mazda_2017")
    pedals = packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 1})
    last_stock = 200  # 100 Hz control frames; the stock radar's last CRZ_INFO lands here
    first_tx = last_stock + STOCK_RADAR_ALIVE_FRAMES + CarControllerParams.LONG_STEP
    panda_armed_at = software_armed_at = None
    for i in range(last_stock + 3 * STOCK_RADAR_GUARD_FRAMES):
      msgs = []
      if i % 2 == 0:  # the 50 Hz PEDALS clock, MRCC main armed from the first frame
        self._rx(self._acc_armed_msg(True))
        msgs.append(pedals)
        if i <= last_stock:
          msgs.append(mazdacan.create_acc_command(packer, 0, i // 2, 0., long_active=False, acc_available=True))
      ret, _ = CI.update([(int(i * DT_CTRL * 1e9), [(m[0], m[1], m[2]) for m in msgs])])
      if i == first_tx:
        self.assertTrue(self._tx(common.make_msg(0, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
      if panda_armed_at is None and self.safety.get_controls_allowed_lateral():
        panda_armed_at = i
      if software_armed_at is None and ret.cruiseState.available:
        software_armed_at = i
    self.assertIsNotNone(panda_armed_at)
    self.assertIsNotNone(software_armed_at)
    self.assertGreater(software_armed_at, panda_armed_at, msg="the software armed MADS before the panda would accept torque")
    # by roughly the margin values.py budgets for PEDALS jitter and pipeline latency
    margin_frames = int(CarControllerParams.STOCK_RADAR_GUARD_MARGIN_T / DT_CTRL)
    self.assertGreaterEqual(software_armed_at - panda_armed_at, margin_frames - 2)
    # and the panda's edge has not been consumed by the time the software arrives
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._torque_cmd_msg(5)))

  def test_camera_bus_radar_tx_does_not_master(self):
    # only the main-bus replay marks mastery; the camera-bus copy is a duplicate
    self.safety.set_mads_params(True, False, False)
    self.assertTrue(self._tx(common.make_msg(2, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
    for _ in range(60):
      self._rx(self._acc_armed_msg(True))
    self.assertFalse(self.safety.get_acc_main_on())

  def test_acc_main_follows_armed_state_after_the_latch(self):
    # after the latch, acc_main_on tracks PEDALS arming both ways (main off must still exit)
    self.safety.set_mads_params(True, False, False)
    self.assertTrue(self._tx(common.make_msg(0, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
    for _ in range(60):
      self._rx(self._acc_armed_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self._rx(self._acc_armed_msg(False))
    self.assertFalse(self.safety.get_acc_main_on())
    self._rx(self._acc_armed_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())

  def test_crz_info_active_gated_on_controls(self):
    # ACC_ACTIVE mirrors CRZ_CTRL's gate: an engaged-claiming accel frame must not flow while
    # controls are not allowed. The body raises PEDALS.ACC_ACTIVE off the SET press before
    # our first engaged frame in every logged engagement, so there is no deadlock.
    for bus in (0, 2):
      for active in (False, True):
        msg = self._accel_msg(self.INACTIVE_ACCEL, bus=bus, active=active)
        self.safety.set_controls_allowed(False)
        self.assertEqual(not active, self._tx(msg))
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._tx(msg))


class TestMazdaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


class TestMazdaTjaMads(unittest.TestCase):
  """The physical TJA button as the MADS lateral switch, declared by the driver.

  The button is fitted to some trims only and neither MAZDA_CX5_2022 nor MAZDA_CX9_2021
  predicts it, so a sunnypilot safety param carries the driver's declaration. Without it every
  car keeps the MRCC-derived main edge and bit 11 is ignored.
  """

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self._init(tja_button=False)

  def _init(self, tja_button):
    self.safety.set_current_safety_param_sp(MazdaSafetyFlagsSP.TJA_BUTTON if tja_button else 0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()
    self.safety.set_mads_params(True, False, False)

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _btns(self, tja=False):
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": tja})

  def _crz_ctrl(self, main_on):
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_AVAILABLE": main_on})

  def test_undeclared_keeps_mrcc_path_and_ignores_the_bit(self):
    self.safety.safety_rx_hook(self._btns(True))
    self.safety.safety_rx_hook(self._btns(False))
    self.assertEqual(-1, self.safety.get_mads_button_press())  # UNAVAILABLE
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_declared_button_allows_lateral_without_mrcc(self):
    self._init(tja_button=True)
    self.safety.safety_rx_hook(self._btns(False))
    self.assertEqual(0, self.safety.get_mads_button_press())  # NOT_PRESSED
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    self.safety.safety_rx_hook(self._btns(True))
    self.assertEqual(1, self.safety.get_mads_button_press())  # PRESSED
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    self.safety.safety_rx_hook(self._btns(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_declared_button_mrcc_does_not_drive_the_main_edge(self):
    self._init(tja_button=True)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    self.safety.safety_rx_hook(self._btns(True))
    self.safety.safety_rx_hook(self._btns(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # MRCC going off produces no falling edge and cannot disengage lateral.
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_declaration_is_read_at_init(self):
    self._init(tja_button=True)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertFalse(self.safety.get_acc_main_on())

    self._init(tja_button=False)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertTrue(self.safety.get_acc_main_on())


if __name__ == "__main__":
  unittest.main()
