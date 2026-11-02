#!/usr/bin/env python3
import unittest

from opendbc.car.mazda.values import CarControllerParams, MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.mazda.values import MazdaSafetyFlagsSP
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg

# both-low PEDALS samples before main falls, as carstate counts them
DEBOUNCE = CarControllerParams.MAIN_OFF_DEBOUNCE_SAMPLES


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  """Upstream's envelope with no safety param bit. The interface no longer emits it for any
  Mazda; the panda keeps it as the default, so it stays proven."""

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

  def _button_msg(self, resume=False, cancel=False, set_m=False, set_p=False, tja=False, bus=0):
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
    return self.packer.make_can_msg_safety("CRZ_BTNS", bus, values)

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
    # 0x243/0x440 whenever openpilot is not steering (stock lane keep, TJA/CTS and dash LDW
    # stay live); steering hands them to openpilot. The fwd hook forwards the camera copy
    # exactly while the tx hook vetoes ours, and vice versa. Under MADS lateral is its own
    # axis, so cruise alone leaves the camera in charge; with MADS off lateral follows cruise
    for mads in (False, True):
      self.safety.set_mads_params(mads, False, False)
      for controls_allowed, controls_allowed_lateral in [(False, False), (True, False), (False, True), (True, True)]:
        stock_active = not (controls_allowed_lateral or (controls_allowed and not mads))
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_controls_allowed_lateral(controls_allowed_lateral)
        for addr, msg in ((0x243, self._torque_cmd_msg(0)), (0x440, self._laneinfo_msg())):
          fwd_bus = 0 if stock_active else -1
          self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(2, addr), f"{mads=} {controls_allowed=} {controls_allowed_lateral=} {addr=:#x}")
          self.assertEqual(not stock_active, self._tx(msg), f"openpilot tx {mads=} {controls_allowed=} {controls_allowed_lateral=} {addr=:#x}")
    self.safety.set_mads_params(False, False, False)

  def _cam_tja_press(self, ctr=5, **overrides):
    # the wheel's idle pattern with the TJA bit: 00 09 ff Cx 00 00 00 00
    values = {"TJA_BUTTON": 1, "DISTANCE_LESS_INV": 1, "BIT1": 1, "BIT2": 1, "BIT3": 1, "CAN_OFF_INV": 1, "RES_INV": 1,
              "SET_P_INV": 1, "SET_M_INV": 1, "DISTANCE_MORE_INV": 1, "MODE_X_INV": 1, "MODE_Y_INV": 1, "CTR": ctr}
    values.update(overrides)
    return self.packer.make_can_msg_safety("CRZ_BTNS", 2, values)

  def test_no_button_reaches_the_camera(self):
    # openpilot never writes CRZ_BTNS on the camera bus: the TJA press there switches the
    # car's own lane-keep setting off (CAM_SETTINGS), after which the EPS applies no LKAS torque
    for mads in (False, True):
      self.safety.set_mads_params(mads, False, False)
      for controls_allowed, controls_allowed_lateral in [(False, False), (True, False), (False, True), (True, True)]:
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_controls_allowed_lateral(controls_allowed_lateral)
        for ctr in range(16):
          self.assertFalse(self._tx(self._cam_tja_press(ctr=ctr)), f"{mads=} {controls_allowed=} {controls_allowed_lateral=} {ctr=}")
        self.assertFalse(self._tx(make_msg(2, 0x09d, 8)))
    self.safety.set_mads_params(False, False, False)

  def test_tja_button_never_pressed_on_the_car_side(self):
    # bit 11 on bus 0 would toggle MADS through the rx hook and arm MRCC in the body
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertFalse(self._tx(self._button_msg(tja=True)))
      self.assertFalse(self._tx(self._button_msg(tja=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(tja=True, cancel=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))


class TestMazdaEpsSafety(TestMazdaSafety):
  """Every gen1 Mazda EPS, steer-to-zero and legacy firmware alike: MazdaSafetyFlags.EPS_HW selects
  the 1200-count envelope with the EPS's own 10/10 slew. The numbers are the controller's, so the
  panda is proven against exactly what the car side runs."""

  SAFETY_PARAM = MazdaSafetyFlags.EPS_HW

  MAX_RATE_UP = CarControllerParams.STEER_DELTA_UP
  MAX_RATE_DOWN = CarControllerParams.STEER_DELTA_DOWN
  MAX_TORQUE_LOOKUP = [0], [CarControllerParams.EPS_STEER_MAX]

  MAX_RT_DELTA = 320

  DRIVER_TORQUE_ALLOWANCE = CarControllerParams.STEER_DRIVER_ALLOWANCE
  DRIVER_TORQUE_FACTOR = CarControllerParams.STEER_DRIVER_MULTIPLIER

  def test_upstream_envelope_without_the_bit(self):
    # with it clear upstream's limits refuse the very first frame above 800.
    # 4, the retired legacy-firmware bit, selects nothing: a car side that still sends it alone
    # gets the stricter limits, never looser ones
    for param in (0, 4):
      with self.subTest(param=param):
        self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, param)
        self.safety.init_tests()
        self.safety.set_controls_allowed(True)
        self._reset_torque_driver_measurement(0)
        self._set_prev_torque(0)
        self.assertFalse(self._tx(self._torque_cmd_msg(TestMazdaSafety.MAX_RATE_UP + 1)))
        self._set_prev_torque(800)
        self.assertFalse(self._tx(self._torque_cmd_msg(801)))

class TestMazdaLongitudinalSafety(TestMazdaEpsSafety, common.LongitudinalAccelSafetyTest):
  """LONG always travels with the EPS bit."""

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0], [0x21b, 0], [0x21c, 0], [0x499, 0],
             [0x361, 0], [0x362, 0], [0x363, 0], [0x364, 0], [0x365, 0], [0x366, 0], [0x764, 0],
             [0x21b, 2], [0x21c, 2], [0x499, 2], [0x361, 2], [0x362, 2], [0x363, 2], [0x364, 2], [0x365, 2], [0x366, 2]]

  SAFETY_PARAM = MazdaSafetyFlags.LONG | MazdaSafetyFlags.EPS_HW

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

    # the tracks carry the controller's counter in byte 7's low nibble; the static frame has none
    self.safety.set_controls_allowed(True)
    for addr, dat in radar_messages.items():
      for counter in range(1, 16):
        counted = dat[:7] + bytes([dat[7] | counter])
        self.assertEqual(addr != 0x499, self._tx(common.make_msg(0, addr, 8, counted)), f"{addr=:#x} {counter=}")

  def test_g46l_radar_static_allowed(self):
    # the 2016.5 G46L body's own static capture; the 2022 one above is not its frame, and
    # the G46L never sends track messages at all
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        self.assertTrue(self._tx(common.make_msg(bus, 0x499, 8, bytes.fromhex("0098400000000000"))))

    self.safety.set_controls_allowed(True)
    for bus in (0, 2):
      self.assertFalse(self._tx(common.make_msg(bus, 0x499, 8, bytes.fromhex("0098400100000000"))))

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

    # no lead is advertised, so an occupied track is never ours, engaged or not
    occupied = [
      "0a4000001c000000",  # stopped lead at 10.25 m, the retired straight-ahead template
      "229e00007c00000e",  # lead at 34.56 m at the retired capture's -8 deg angle
    ]
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        for addr in range(0x361, 0x367):
          for hexdat in occupied:
            self.assertFalse(self._tx(common.make_msg(bus, addr, 8, bytes.fromhex(hexdat))))

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

  def _pedals_msg(self, armed, brake=0):
    # PEDALS with MRCC armed-but-idle (ACC_OFF), the state that persists across ignition
    values = {"ACC_OFF": armed, "BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _armed(self):
    # one armed PEDALS sample is main: no radar takeover, no synthetic frame, no latch
    self.safety.set_mads_params(True, False, False)
    self._rx(self._pedals_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_acc_main_follows_armed_state_from_boot(self):
    # MADS lateral needs only the main switch; the radar takeover gates cruise (controls_allowed
    # needs a SET-qualified engaged edge)
    self._armed()
    self.assertTrue(self._tx(self._torque_cmd_msg(5)))
    self.assertFalse(self.safety.get_controls_allowed())

  def _main_off(self, samples, brake):
    for _ in range(samples):
      self._rx(self._pedals_msg(armed=False, brake=brake))

  def test_acc_main_follows_armed_state_both_ways(self):
    # acc_main_on tracks PEDALS arming both ways (main off must still exit)
    self._armed()
    self._main_off(DEBOUNCE, brake=False)
    self.assertFalse(self.safety.get_acc_main_on())
    self._rx(self._pedals_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_main_off_lands_after_the_debounce(self):
    # brake or no brake: a cancel mashed under braking (route 7f9e3ff336), the KE's MODE_X
    # main-off at a stop (route_ke_0b), and a main-off with no visible button all land
    for brake in (False, True):
      self._armed()
      self._main_off(DEBOUNCE - 1, brake)
      self.assertTrue(self.safety.get_acc_main_on())
      self.assertTrue(self.safety.get_controls_allowed_lateral())
      self._main_off(1, brake)
      self.assertFalse(self.safety.get_acc_main_on())
      self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_short_dropout_is_held(self):
    self._armed()
    for _ in range(3):
      self._main_off(DEBOUNCE - 1, brake=True)
      self.assertTrue(self.safety.get_acc_main_on())
      self._rx(self._pedals_msg(armed=True, brake=True))

  def test_main_toggled_at_a_stop_re_arms(self):
    # route 000001c9--0b2a64a214 seg 0: main toggled at a red light with the brake held. If
    # main does not fall here, the next press has no rising edge, lateral never re-arms, and
    # MADS runs into Controls Mismatch: Lateral
    self._armed()
    self._main_off(DEBOUNCE, brake=True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx(self._pedals_msg(armed=True, brake=True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_cancel_exits_controls_at_once(self):
    self._armed()
    self.safety.set_controls_allowed(True)
    self._rx(self._button_msg(cancel=True))
    self.assertFalse(self.safety.get_controls_allowed())
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


class MazdaTjaButtonBase(unittest.TestCase):
  """MADS on, the TJA button declared (or not) through the sunnypilot safety param."""

  TJA_BUTTON = False

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self._init(tja_button=self.TJA_BUTTON)

  def _init(self, tja_button, param=0):
    self.safety.set_current_safety_param_sp(MazdaSafetyFlagsSP.TJA_BUTTON if tja_button else 0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, param)
    self.safety.init_tests()
    self.safety.set_mads_params(True, False, False)

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)
    self.safety.set_mads_params(False, False, False)

  def _crz_ctrl(self, main_on):
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_AVAILABLE": main_on})

  def _pedals(self, acc_off, brake=False):
    return self.packer.make_can_msg_safety("PEDALS", 0, {"ACC_OFF": acc_off, "BRAKE_ON": brake})


class TestMazdaTjaMads(MazdaTjaButtonBase):
  """The physical TJA button as the MADS lateral switch, declared by the driver.

  The button is fitted to some trims only and neither MAZDA_CX5_2022 nor MAZDA_CX9_2021
  predicts it, so a sunnypilot safety param carries the driver's declaration. Declared, bit 11
  drives the MADS button and MRCC no longer touches the main edge in either direction: its
  falling edge would otherwise exit the panda's lateral while the software's MADS stays on.
  Undeclared cars keep the MRCC-derived main edge and bit 11 is ignored.
  """

  def _btns(self, tja=False):
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": tja})

  def test_undeclared_keeps_mrcc_path_and_ignores_the_bit(self):
    self.safety.safety_rx_hook(self._btns(True))
    self.safety.safety_rx_hook(self._btns(False))
    self.assertEqual(0, self.safety.get_mads_button_press())  # NOT_PRESSED
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

    # the MRCC master button (route 00000018 seg 12) disarms MRCC and the camera, not MADS:
    # no falling edge here, so the panda's lateral stays with the software's
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_declared_button_under_openpilot_longitudinal(self):
    # the PEDALS-derived main edge is guarded the same way
    self._init(tja_button=True, param=MazdaSafetyFlags.LONG | MazdaSafetyFlags.EPS_HW)
    self.safety.safety_rx_hook(self._pedals(True))
    self.assertFalse(self.safety.get_acc_main_on())
    self.safety.safety_rx_hook(self._btns(True))
    self.safety.safety_rx_hook(self._btns(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.safety_rx_hook(self._pedals(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_declaration_is_read_at_init(self):
    self._init(tja_button=True)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertFalse(self.safety.get_acc_main_on())

    self._init(tja_button=False)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertTrue(self.safety.get_acc_main_on())


class TestMazdaMainOnMads(MazdaTjaButtonBase):
  def _btns(self, on=False, off=False, bus=0):
    return self.packer.make_can_msg_safety("CRZ_BTNS", bus, {"MODE_Y": on, "MODE_X": off})

  def _main(self, on, longitudinal):
    return self._pedals(on) if longitudinal else self._crz_ctrl(on)

  def _standby(self, longitudinal=False, tja=False):
    self._init(tja_button=tja, param=MazdaSafetyFlags.LONG if longitudinal else 0)
    self.safety.safety_rx_hook(self._main(True, longitudinal))
    self.safety.safety_rx_hook(self._btns())
    # The boot-time main edge expires before selfdrived sends an engaged heartbeat.
    self._expire_lateral()

  def _expire_lateral(self):
    self.safety.set_heartbeat_engaged_mads(False)
    for _ in range(3):
      self.safety.mads_heartbeat_engaged_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_on_rearms_standby_without_engaging_cruise(self):
    for longitudinal in (False, True):
      with self.subTest(longitudinal=longitudinal):
        self._standby(longitudinal)
        self.safety.safety_rx_hook(self._btns(on=True))
        self.assertTrue(self.safety.get_controls_allowed_lateral())
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertTrue(self.safety.get_acc_main_on())
        self.safety.safety_rx_hook(self._btns())
        for _ in range(DEBOUNCE if longitudinal else 1):
          self.safety.safety_rx_hook(self._main(False, longitudinal))
        self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_held_on_does_not_rearm_after_heartbeat_exit(self):
    self._standby()
    self.safety.safety_rx_hook(self._btns(on=True))
    self._expire_lateral()
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.safety.safety_rx_hook(self._btns())
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_off_and_camera_bus_do_not_rearm(self):
    self._standby()
    for msg in (self._btns(off=True), self._btns(), self._btns(on=True, bus=2)):
      self.safety.safety_rx_hook(msg)
      self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_requires_main_on_and_mads_enabled(self):
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._standby()
    self.safety.set_mads_params(False, False, False)
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_declared_tja_ignores_on(self):
    self._standby(tja=True)
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_pause_on_brake_keeps_request_until_release(self):
    self._standby()
    self.safety.set_mads_params(True, False, True)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.safety.safety_rx_hook(self._pedals(False, brake=True))
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self.safety.get_controls_requested_lateral())
    self.safety.safety_rx_hook(self._pedals(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())


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


class TestMazdaMrccOffCleanup(MazdaTjaButtonBase):
  """The exact-bytes MRCC master tap that undoes the arm a physical TJA press causes."""

  # Nonzero signals only; unlisted DBC signals pack as 0 (see create_mrcc_off_cmd).
  MRCC_OFF_VALUES = {
    "CAN_OFF_INV": 1, "SET_P_INV": 1, "RES_INV": 1, "SET_M_INV": 1,
    "DISTANCE_LESS_INV": 1, "DISTANCE_MORE_INV": 1, "MODE_X_INV": 1, "MODE_Y_INV": 1,
    "BIT1_INV": 1, "BIT2": 1, "BIT3": 1, "CTR": 4,
  }
  TJA_BUTTON = True

  def _mrcc_off(self, **over):
    values = dict(self.MRCC_OFF_VALUES)
    values.update(over)
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_exact_tap_allowed_declared_and_armed(self):
    # not controlling: this is the whole point of the exception
    self.safety.safety_rx_hook(self._pedals(True))
    self.assertTrue(self.safety.safety_tx_hook(self._mrcc_off()))

  def test_exact_tap_blocked_undeclared(self):
    self._init(tja_button=False)
    self.safety.safety_rx_hook(self._pedals(True))
    self.assertFalse(self.safety.safety_tx_hook(self._mrcc_off()))

  def test_exact_tap_blocked_when_cruise_not_armed(self):
    self.safety.safety_rx_hook(self._pedals(False))
    self.assertFalse(self.safety.safety_tx_hook(self._mrcc_off()))

  def test_arm_reads_pedals_not_the_radar_under_stock_long(self):
    # carstate sends on PEDALS; the radar's CRZ_CTRL bit lags it and must not be the gate
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertFalse(self.safety.safety_tx_hook(self._mrcc_off()))
    self.safety.safety_rx_hook(self._pedals(True))
    self.assertTrue(self.safety.safety_tx_hook(self._mrcc_off()))
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.assertTrue(self.safety.safety_tx_hook(self._mrcc_off()))
    self.safety.safety_rx_hook(self._pedals(False))
    self.assertFalse(self.safety.safety_tx_hook(self._mrcc_off()))

  def test_armed_tracks_pedals_after_teardown(self):
    self._init(tja_button=True, param=MazdaSafetyFlags.LONG | MazdaSafetyFlags.EPS_HW)
    self.safety.safety_rx_hook(self._pedals(True))
    self.assertTrue(self.safety.safety_tx_hook(self._mrcc_off()))
    self.safety.safety_rx_hook(self._pedals(False))
    self.assertFalse(self.safety.safety_tx_hook(self._mrcc_off()))

  def test_lookalike_tap_is_blocked_everywhere(self):
    self.safety.safety_rx_hook(self._pedals(True))
    for over in ({"SET_P": 1}, {"RES": 1}, {"TJA_BUTTON": 1}, {"BIT2": 0}, {"CTR": 12}):
      with self.subTest(over=over):
        msg = self._mrcc_off(**over)
        # only the CTR variation stays valid; anything else is a composite command
        allowed = over == {"CTR": 12}
        self.assertEqual(allowed, self.safety.safety_tx_hook(msg))

  def test_cancel_still_allowed_while_not_controlling(self):
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    msg = self.packer.make_can_msg_safety("CRZ_BTNS", 0, {"CAN_OFF": 1})
    self.assertTrue(self.safety.safety_tx_hook(msg))

  def test_tja_bit_still_blocked_on_the_main_bus(self):
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    msg = self.packer.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": 1})
    self.assertFalse(self.safety.safety_tx_hook(msg))


if __name__ == "__main__":
  unittest.main()
