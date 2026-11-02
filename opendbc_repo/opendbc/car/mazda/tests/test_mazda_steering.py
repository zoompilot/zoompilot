"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Steering on the 2022+ EPS: the torque parameters (gated on the EPS, not the model), the
EPS ceiling and rail, the driver-torque headroom against the panda's window, and the
non-delivery latch's zeroing of the command.
"""
from collections import deque

import numpy as np
import pytest

from opendbc.can import CANPacker
from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.mazda.tests.conftest import CAM_LKAS, LongCtrlState, car_controller, car_params, controller_params, eps_fw, frame, \
  mazda_car_state, step
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py

SWAPPED_EPS_FW = eps_fw(b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
LEGACY_FW_EPS = eps_fw(b'K319-3210X-B-00' + b'\x00' * 9)  # THACO CX-5 2023, keeps the floor
PANDA_DRIVER_SAMPLES = 6  # the panda's driver-torque window, MAX_SAMPLE_VALS in safety/declarations.h


class Panda:
  """The compiled safety model on the EPS envelope, controls allowed, for closed-loop runs of the
  real controller: each frame the panda samples the driver, then judges the 0x243 we send."""

  def __init__(self):
    self.safety = libsafety_py.libsafety
    self.packer = CANPacker("mazda_2017")
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.EPS_HW)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    for _ in range(PANDA_DRIVER_SAMPLES):
      self.driver(0, 0)

  def packet(self, name: str, values: dict):
    addr, dat, bus = self.packer.make_can_msg(name, 0, values)
    return libsafety_py.make_CANPacket(addr, bus, dat)

  def driver(self, frame_idx: int, torque: float) -> None:
    self.safety.set_timer(frame_idx * 10_000)
    self.safety.safety_rx_hook(self.packet("STEER_TORQUE", {"STEER_TORQUE_SENSOR": torque}))

  def tx(self, dat: bytes) -> bool:
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(CAM_LKAS, 0, dat))

  def tx_torque(self, torque: int) -> bool:
    return self.safety.safety_tx_hook(self.packet("CAM_LKAS", {"LKAS_REQUEST": torque}))

  def zero_reference(self) -> None:
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)


def cx5_2022_params():
  return controller_params(CAR.MAZDA_CX5_2022)


def eps_swap_params():
  # A CX-5 2022+ EPS swapped into (or shared by) another Mazda: different model, same EPS.
  return controller_params(CAR.MAZDA_CX9_2021, car_fw=SWAPPED_EPS_FW)


def pre_2022_params():
  # a pre-2022 platform with its stock EPS: the hardware envelope behind the 45 kph floor
  return controller_params(CAR.MAZDA_CX5)


def legacy_fw_params():
  # the 2022 EPS hardware on firmware that keeps the floor: measured envelope, 800 scale
  return controller_params(CAR.MAZDA_CX5_2022, car_fw=LEGACY_FW_EPS)


class TestCarControllerParams:

  def test_eps_ceiling_never_exceeds_steer_max_scale(self):
    params = cx5_2022_params()
    # The ceiling is a clamp on delivered-torque counts; the scale is STEER_MAX. The clamp is
    # only meaningful if it sits at or below the scale at every speed.
    assert 0 < min(params.EPS_CEILING_LOOKUP[1]) and max(params.EPS_CEILING_LOOKUP[1]) <= params.STEER_MAX

  def test_eps_ceiling_is_monotone_and_matches_the_measured_rails(self):
    params = cx5_2022_params()
    # Measured over 11.4M clean frames: 1148 below 18 mph, a monotone rolloff, hard 620 from
    # 32.5 mph up (docs/mazda-lkas-camera-tx-census.md). Nothing above 620 was ever delivered
    # above 32.5 mph in 7.5M frames, so the high-speed leg must not drift back up.
    bp, vals = params.EPS_CEILING_LOOKUP
    assert list(vals) == sorted(vals, reverse=True), "ceiling must fall monotonically with speed"
    assert np.interp(5.0, bp, vals) == 1148
    assert np.interp(14.5, bp, vals) == 620
    assert np.interp(35.0, bp, vals) == 620

  def test_steer_delta_matches_the_eps_rate_limit_at_this_steer_step(self):
    params = cx5_2022_params()
    # Per-frame controller deltas must match the EPS hardware slew in both directions: 12 counts
    # per 12 ms 0x241 frame, ~1000 counts/s (slope fits on 1157 saturated ramps: 921-934/s).
    rate_hz = 1.0 / DT_CTRL / CarControllerParams.STEER_STEP
    assert params.STEER_DELTA_UP * rate_hz == pytest.approx(1000, rel=0.01)
    assert params.STEER_DELTA_DOWN * rate_hz == pytest.approx(1000, rel=0.01)

  def test_cx5_2022_steer_max_is_flat(self):
    # one scale at every speed: the EPS is linear in counts, and a step would put the learned
    # torque parameters in two units (docs/zoompilot/lateral-tune.md)
    assert cx5_2022_params().STEER_MAX == 1200

  def test_cx5_2022_rate_limits(self):
    params = cx5_2022_params()
    assert params.STEER_DELTA_UP == 10
    assert params.STEER_DELTA_DOWN == 10

  @pytest.mark.parametrize("params", [cx5_2022_params, eps_swap_params, pre_2022_params, legacy_fw_params],
                           ids=["cx5_2022", "eps_swap", "pre_2022", "legacy_fw"])
  def test_rate_limits_equal_the_pandas_for_each_eps(self, params):
    # The panda's driver_limit_check rejects any frame that retreats by less than max_rate_down
    # once the driver bound is below the last command, and any frame that climbs by more than
    # max_rate_up, so "tighter than the panda" is not allowed: the controller's deltas must
    # equal the panda's for the EPS class it is driving. Route 00000148 lost 171 consecutive
    # frames to a 12-count retreat against a 25-count requirement. The panda's safety tests
    # (TestMazdaEpsSafety) take their envelope from CarControllerParams and prove it against the
    # compiled safety model, so every EPS must run exactly those constants.
    params = params()
    assert params.STEER_DELTA_UP == CarControllerParams.STEER_DELTA_UP
    assert params.STEER_DELTA_DOWN == CarControllerParams.STEER_DELTA_DOWN
    assert params.STEER_MAX == CarControllerParams.EPS_STEER_MAX
    assert params.STEER_DRIVER_MULTIPLIER == CarControllerParams.STEER_DRIVER_MULTIPLIER
    assert params.STEER_DRIVER_ALLOWANCE == CarControllerParams.STEER_DRIVER_ALLOWANCE

  def test_cx5_eps_driver_multiplier(self):
    # 15 is the CX-5-EPS tune (upstream stock is 1)
    assert cx5_2022_params().STEER_DRIVER_MULTIPLIER == 15

  def test_eps_swap_gets_cx5_tune(self):
    params = eps_swap_params()
    # EPS present (STEER_TO_ZERO_EPS) on a non-CX-5 model still gets the higher-authority tune
    assert params.STEER_MAX == 1200
    assert params.STEER_DRIVER_MULTIPLIER == 15

  @pytest.mark.parametrize("params", [legacy_fw_params, pre_2022_params], ids=["legacy_fw_in_2022_body", "pre_2022_platform"])
  def test_legacy_firmware_gets_the_same_envelope_and_tune(self, params):
    legacy, stz = params(), cx5_2022_params()
    for attr in ('STEER_MAX', 'EPS_CEILING_LOOKUP', 'STEER_DELTA_UP', 'STEER_DELTA_DOWN',
                 'STEER_DRIVER_MULTIPLIER', 'STEER_DRIVER_SAMPLES', 'STEER_DRIVER_MARGIN'):
      assert getattr(legacy, attr) == getattr(stz, attr), attr
    # the latch reads LKAS_TRACK_STATE semantics only the steer-to-zero firmware has
    assert not hasattr(legacy, 'STEER_UNDELIVERED_FRAMES')

  def test_undelivered_threshold_clears_normal_operation(self):
    # 20 frames is an order of magnitude clear of both populations: across 96k unblocked
    # frames with |request| > 200 the longest run of LKAS_EFFECTIVE == 0 is 2 frames, while
    # blocked runs reach 183. Derivation: tools/mazda_long/analyze_lkas_nondelivery.py
    params = cx5_2022_params()
    assert params.STEER_UNDELIVERED_FRAMES > 2 * 5
    assert params.STEER_UNDELIVERED_FRAMES < 183 // 2
    # the request has to clear the rate limiter's walk before the count can start, so the
    # minimum must sit above what one STEER_DELTA_UP step delivers
    assert params.STEER_UNDELIVERED_MIN > params.STEER_DELTA_UP

  def test_the_alert_thresholds_sit_between_the_benign_and_faulting_populations(self):
    # The camera latches CAM_LKAS.ERR_BIT_1 on a budget of LKAS requests the EPS never applies
    # (route 00000139 seg 14). carstate owns the latch (test_mazda_carstate.py); the controller
    # obeys it (test_carstate_undelivered_latch_zeroes_the_steer_command below).
    params = cx5_2022_params()
    # above the speed gate, non-delivery runs reach 30 frames on every route that never
    # faulted and 315 on the two that did, so the hold has to clear the first and not the
    # second. Same separation argument as STEER_UNDELIVERED_FRAMES itself.
    total = params.STEER_UNDELIVERED_FRAMES + params.STEER_UNDELIVERED_ALERT_FRAMES
    assert total > 30 * (100 / 85)
    assert total < 315 * (100 / 85)
    # Honda's per-car low-speed alert minimums span 2-15 mph; ours has to sit in that range,
    # above the block's own release band (p90 4.97 m/s) so a creep-away cannot arm the alert
    # on its way out, and below where route 148's fault began (5.9 m/s)
    assert params.STEER_UNDELIVERED_ALERT_MIN_SPEED > 4.97
    assert params.STEER_UNDELIVERED_ALERT_MIN_SPEED < 5.9
    assert params.STEER_UNDELIVERED_ALERT_MIN_SPEED < 15. * CV.MPH_TO_MS
    # 1660 of 1915 LKAS_BLOCK episodes in 64 h begin below 0.5 m/s (the EPS's standby from a
    # stop, read through wheel-speed quantisation); every latched block that began above it
    # and armed the alert was a fault, the slowest of them route 00000148's at 4.6 m/s
    # (tools/mazda_long/replay_undelivered_alert.py)
    assert params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED > 0.5
    assert params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED < 4.6


def test_carstate_undelivered_latch_zeroes_the_steer_command(stock_cc, stock_cs):
  # carstate owns the latch (STEER_RATE request vs delivery); the controller only obeys it
  lat = dict(long_active=False, enabled=True, lat_active=True, torque=1.0, v_ego=10.)
  for _ in range(50):
    step(stock_cc, stock_cs, **lat)
  assert stock_cc.apply_torque_last > stock_cc.params.STEER_UNDELIVERED_MIN
  step(stock_cc, stock_cs, steer_undelivered=True, **lat)
  assert stock_cc.apply_torque_last == 0
  # the latch clearing walks the command back up from zero at STEER_DELTA_UP
  step(stock_cc, stock_cs, steer_undelivered=False, **lat)
  assert stock_cc.apply_torque_last == stock_cc.params.STEER_DELTA_UP


class TestRejectionRecovery:
  """A panda rejection resets its rate-limit reference to zero, so a controller that keeps ramping is
  rejected on every later frame and the EPS loses its 0x243 stream (route 00000148: 1.72 s, route
  00000139: 0.75 s, drive_02: 0.63 s). About 0.6 s in the EPS raises LKAS_FAULT and the camera
  faults 5.3 s later. The panda reports every frame it refused back on the can stream, carstate
  counts the torque requests among them, and the controller restarts its ramp from zero, which is
  the one place the panda accepts next."""

  LAT = dict(long_active=False, enabled=True, lat_active=True, torque=1.0, v_ego=10.)

  def test_no_report_never_restarts_the_ramp(self, stock_cc, stock_cs):
    for _ in range(60):
      actuators, _ = step(stock_cc, stock_cs, lkas_rejected=0, **self.LAT)
    assert actuators.torqueOutputCan == 60 * stock_cc.params.STEER_DELTA_UP

  def test_a_reported_rejection_restarts_from_one_step_of_zero(self, stock_cc, stock_cs):
    params = stock_cc.params
    for _ in range(30):
      actuators, _ = step(stock_cc, stock_cs, lkas_rejected=0, **self.LAT)
    assert actuators.torqueOutputCan == 30 * params.STEER_DELTA_UP
    actuators, _ = step(stock_cc, stock_cs, lkas_rejected=1, **self.LAT)
    assert actuators.torqueOutputCan == params.STEER_DELTA_UP  # one step from zero
    # delivery resumes and the ramp rebuilds from there
    for i in range(2, 10):
      actuators, _ = step(stock_cc, stock_cs, lkas_rejected=0, **self.LAT)
      assert actuators.torqueOutputCan == i * params.STEER_DELTA_UP

  def test_every_reported_rejection_restarts_again(self, stock_cc, stock_cs):
    # a stream the panda keeps refusing (its lateral not armed) holds at one step, never ramps
    # blind to the rail
    for _ in range(20):
      actuators, _ = step(stock_cc, stock_cs, lkas_rejected=1, **self.LAT)
      assert actuators.torqueOutputCan == stock_cc.params.STEER_DELTA_UP

  def test_a_rejection_while_commanding_zero_changes_nothing(self, stock_cc, stock_cs):
    for _ in range(20):
      actuators, _ = step(stock_cc, stock_cs, lkas_rejected=1, long_active=False, enabled=False, lat_active=False, v_ego=10.)
      assert actuators.torqueOutputCan == 0
    assert stock_cc.apply_torque_last == 0
    actuators, _ = step(stock_cc, stock_cs, lkas_rejected=0, **self.LAT)
    assert actuators.torqueOutputCan == stock_cc.params.STEER_DELTA_UP

  def test_the_restart_applies_on_every_envelope(self):
    # the panda's reset is the same whatever the EPS, so the legacy platforms restart too
    cc = car_controller(alpha_long=False, candidate=CAR.MAZDA_CX9_2021)
    cs = mazda_car_state(cc.CP, cc.CP_SP)
    for _ in range(30):
      step(cc, cs, lkas_rejected=0, **self.LAT)
    assert cc.apply_torque_last == 30 * cc.params.STEER_DELTA_UP
    actuators, _ = step(cc, cs, lkas_rejected=1, **self.LAT)
    assert actuators.torqueOutputCan == cc.params.STEER_DELTA_UP

  # Closed loop through the compiled safety model: the panda's report reaches CarState
  # report_delay cycles after the refused frame (it rides the can stream through pandad, one or
  # two card cycles behind on the device).

  STALE_FRAMES = 8

  def controller_loop(self, panda, frames, driver_seen_by_controller, report_delay=1, report=True):
    """frames yields the driver torque the panda samples; the controller sees
    driver_seen_by_controller(frame) instead, so a stale sample can be staged. Returns (accepted
    torques by frame, longest run of rejected frames)."""
    cc = self.cc
    refused = deque([0] * report_delay, maxlen=report_delay)
    accepted, rejected_run, longest = [], 0, 0
    for i, driver_torque in enumerate(frames, start=1):
      panda.driver(i, driver_torque)
      _, sends = step(cc, self.cs, driver_torque=driver_seen_by_controller(i), lkas_rejected=refused[0] if report else 0, **self.LAT)
      dat = frame(sends, CAM_LKAS)
      torque = (((dat[0] & 0x0f) << 8) | dat[1]) - 2048
      if panda.tx(dat):
        accepted.append((i, torque))
        rejected_run = 0
        refused.append(0)
      else:
        rejected_run += 1
        longest = max(longest, rejected_run)
        refused.append(1 if torque != 0 else 0)
    return accepted, longest

  @classmethod
  def stale_driver_sample(cls, frame_idx):
    # what the controller sees of the -100 push on frames 61 to 72: nothing for eight frames
    return -100 if 60 + cls.STALE_FRAMES < frame_idx <= 72 else 0

  @pytest.fixture(autouse=True)
  def _rig(self, stock_cc, stock_cs):
    self.cc, self.cs = stock_cc, stock_cs

  @pytest.mark.parametrize("report_delay", [1, 2, 3])
  def test_controller_recovers_the_stream_after_a_rejection(self, report_delay):
    # 60 frames ramping clean; the driver then pushes -100 for 12 frames, which the panda's
    # 6-sample window sees at once while the controller's sample runs 8 frames stale (the route
    # 00000148 staleness); then both agree again. With the panda's own report the controller
    # restarts from zero as soon as it arrives.
    frames = [0] * 60 + [-100] * 12 + [0] * 120
    accepted, longest = self.controller_loop(Panda(), frames, self.stale_driver_sample, report_delay)
    assert longest > 0, "the stale sample must reject at least one frame"
    # the restart lands one report behind the refusal, and each restart is refused again while
    # the controller's driver sample is still stale, so the outage is the staleness plus the
    # report delay: a sixth of the EPS's 0.6 s timeout at the slowest report
    assert longest <= self.STALE_FRAMES + report_delay + 1
    # and the ramp rebuilds to the rail afterwards
    assert accepted[-1][1] == accepted[-2][1]
    assert accepted[-1][1] > 500

  @pytest.mark.parametrize("report_delay", [1, 2, 3])
  def test_a_lone_rejection_costs_only_the_report_delay(self, report_delay):
    # the controller's sample fresh: the panda's reference zeroed from outside after 60 clean
    # frames (a disengage-and-arm blip the controller never saw), so the next command is far
    # above one step; the outage is exactly the time the report takes to come back
    panda = Panda()
    self.controller_loop(panda, [0] * 60, lambda f: 0, report_delay)
    panda.zero_reference()
    accepted, longest = self.controller_loop(panda, [0] * 140, lambda f: 0, report_delay)
    assert longest == report_delay
    assert accepted[-1][1] > 500

  def test_without_the_rejection_report_a_rejection_starves_the_eps(self):
    # the same scenario with no report is the failure the captures show: rejected to the end
    frames = [0] * 60 + [-100] * 12 + [0] * 120
    _, longest = self.controller_loop(Panda(), frames, self.stale_driver_sample, report=False)
    assert longest >= 60, "the EPS 0x243 timeout is about 60 frames"


class TestDriverTorqueHeadroom:
  """The panda enforces the same driver-torque envelope from the min/max of its own last 6
  STEER_TORQUE samples, while the controller sees one sample that is already a control cycle
  old. At a multiplier of 15 that staleness is worth 15 counts of ceiling per count of driver
  torque, and route 00000148 lost 1721 ms of LKAS delivery to it."""

  ALLOWANCE = CarControllerParams.STEER_DRIVER_ALLOWANCE
  MULTIPLIER = 15

  @staticmethod
  def drive(cc, cs, torques, sign=1.0):
    """Feed a driver-torque sequence, commanding hard the whole way, and return the command."""
    out = None
    for dt in torques:
      actuators, _ = step(cc, cs, long_active=False, enabled=False, accel=0., long_state=LongCtrlState.off,
                          lat_active=True, torque=sign, v_ego=6.0, driver_torque=dt, steering_pressed=True)
      out = actuators.torqueOutputCan
    return out

  def panda_ceiling(self, window, steer_max):
    # driver_limit_check: max_torque + (allowance + torque_driver.max) * multiplier
    return steer_max + (self.ALLOWANCE + max(window)) * self.MULTIPLIER

  def test_the_margin_holds_the_ceiling_clear_of_the_pandas(self):
    params = cx5_2022_params()
    # the window's overlap argument dies once the controller falls a full panda window
    # behind, so the margin is what covers the rest. Replay put the requirement at 2 counts.
    assert params.STEER_DRIVER_MARGIN >= 2
    # and it must stay small enough to be a margin rather than a torque cut
    assert params.STEER_DRIVER_MARGIN * self.MULTIPLIER < 0.1 * params.STEER_MAX

  def test_command_stays_under_the_panda_ceiling_while_the_driver_fights(self, cc, cs):
    params = cx5_2022_params()
    # the recorded run: driver torque walking more negative while we command hard positive.
    # Every frame here was rejected on car, starving the EPS of 0x243 entirely.
    seq = [-25, -25, -25, -27, -28, -29, -30, -28, -26, -26, -29, -29, -31, -31, -31]
    out = self.drive(cc, cs, [-20] * 20 + seq)
    # the panda's window holds only the last 6 samples, so its ceiling uses the least
    # adverse of those -- the controller must stay at or below it
    assert out <= self.panda_ceiling(seq[-6:], params.STEER_MAX)

  def test_a_steady_driver_torque_costs_nothing(self, cc, cs):
    # the window only bites when the samples disagree; a constant hand on the wheel must
    # produce exactly what the single-sample limiter always did
    windowed = self.drive(cc, cs, [-10] * 40)
    cc2 = car_controller(alpha_long=True)
    assert windowed == self.drive(cc2, mazda_car_state(cc2.CP, cc2.CP_SP), [-10] * 40)
    assert windowed > 0

  def test_the_adverse_extreme_follows_the_commanded_direction(self, cc, cs):
    # commanding negative, the binding bound is the low one, so the window's high end is
    # the adverse extreme -- picking the same end for both signs would give away authority
    seq = [30, 30, 30, 25, 20, 15, 10, 5, 0, 0]
    out = self.drive(cc, cs, [30] * 20 + seq, sign=-1.0)
    params = cx5_2022_params()
    assert out >= -params.STEER_MAX + (-self.ALLOWANCE + min(seq[-6:])) * self.MULTIPLIER

  @staticmethod
  def limiter_loop(panda, params, frames, ctrl_last=0, frame0=0):
    """The real controller limiter frame by frame through the compiled safety model. frames yields
    (driver_torque, target); returns (rejected frames, full retreats, last command)."""
    rejected, full_retreats = [], 0
    for i, (driver_torque, target) in enumerate(frames, start=frame0 + 1):
      panda.driver(i, driver_torque)
      cmd = apply_driver_steer_torque_limits(target, ctrl_last, driver_torque, params, params.STEER_MAX)
      if not panda.tx_torque(cmd):
        rejected.append((i, cmd, ctrl_last, driver_torque))
      full_retreats += abs(cmd) == abs(ctrl_last) - params.STEER_DELTA_DOWN
      ctrl_last = cmd
    return rejected, full_retreats, ctrl_last

  @pytest.mark.parametrize("slope", [1, 2, 5, 10, 30])
  @pytest.mark.parametrize("sign", [1, -1])
  def test_driver_override_winddown_is_never_rejected(self, sign, slope):
    # The command is held at full torque while the driver ramps against it; the driver bound then
    # falls faster than the controller retreats, so the panda's max_rate_down requirement binds.
    # Route 00000148 lost 171 consecutive frames here when the panda demanded 25 and the
    # controller retreated 12.
    params, panda = cx5_2022_params(), Panda()
    max_torque = params.STEER_MAX
    # ramp up to full torque with no driver input
    ramp = [(0, max_torque * sign)] * (max_torque // params.STEER_DELTA_UP + 5)
    rejected, _, last = self.limiter_loop(panda, params, ramp)
    assert rejected == []
    assert last == max_torque * sign
    # the driver pushes back, harder each frame, to the top of the 8-bit sensor field
    push = [(-sign * min(slope * f, 127), max_torque * sign) for f in range(300)]
    rejected, full_retreats, _ = self.limiter_loop(panda, params, push, ctrl_last=last, frame0=len(ramp))
    assert rejected == [], f"{len(rejected)} frames rejected, first {rejected[:1]}"
    # the scenario only proves something if the bound outran the retreat at least once
    if slope * params.STEER_DRIVER_MULTIPLIER > params.STEER_DELTA_DOWN:
      assert full_retreats > 0


class TestDriverOverride:
  """A push against the command keeps the same share of it at every speed."""

  @staticmethod
  def settle(v_ego, driver_torque):
    cc = car_controller(alpha_long=False)
    cs = mazda_car_state(cc.CP, cc.CP_SP)
    for _ in range(200):
      actuators, _ = step(cc, cs, long_active=False, enabled=True, lat_active=True, torque=0.3,
                          v_ego=v_ego, driver_torque=driver_torque, steering_pressed=driver_torque != 0)
    return actuators.torqueOutputCan

  @pytest.mark.parametrize("v_ego", [3., 15., 30.])
  def test_the_same_push_keeps_the_same_share_at_every_speed(self, v_ego):
    full = self.settle(v_ego, 0)
    assert self.settle(v_ego, -15) == full
    assert self.settle(v_ego, -32.5) == pytest.approx(full * 0.5, abs=1)
    assert self.settle(v_ego, -50) == 0

  def test_a_push_with_the_command_keeps_all_of_it(self):
    assert self.settle(30., 40) == self.settle(30., 0)


def test_carstate_first_engage_hold_zeroes_the_steer_command(stock_cc, stock_cs):
  # carstate derives the hold (first engagement of the cycle, standby, crawl); the controller only obeys it
  lat = dict(long_active=False, enabled=True, lat_active=True, torque=-1.0, v_ego=0.3)
  actuators, _ = step(stock_cc, stock_cs, steer_first_engage_hold=True, **lat)
  assert actuators.torqueOutputCan == 0
  assert stock_cc.apply_torque_last == 0
  # released, the command walks up from zero at STEER_DELTA_UP
  for i in range(1, 4):
    actuators, _ = step(stock_cc, stock_cs, steer_first_engage_hold=False, **lat)
    assert actuators.torqueOutputCan == -i * stock_cc.params.STEER_DELTA_UP


def test_the_first_engage_hold_is_the_steer_to_zero_eps_only(stock_cc, stock_cs):
  stock_cc.steer_to_zero = False
  actuators, _ = step(stock_cc, stock_cs, steer_first_engage_hold=True, long_active=False, enabled=True, lat_active=True, torque=-1.0, v_ego=0.3)
  assert actuators.torqueOutputCan == -stock_cc.params.STEER_DELTA_UP


class TestTorqueTune:
  @pytest.mark.parametrize("platform", [CAR.MAZDA_CX9_2021, CAR.MAZDA_CX5_2022])
  def test_tune_converted_to_steer_max(self, platform):
    # params.toml's fit (or the CX-5 2022's own) is on upstream's 800 counts: the same counts per
    # m/s^2 at 1200
    from opendbc.car.interfaces import get_torque_params
    from opendbc.car.mazda.values import TORQUE_TUNES
    toml = get_torque_params()[platform]
    laf, friction = TORQUE_TUNES.get(platform, (toml['LAT_ACCEL_FACTOR'], toml['FRICTION']))
    tune = car_params(platform).lateralTuning.torque
    assert tune.latAccelFactor == pytest.approx(laf * 1.5, rel=1e-6)
    assert tune.friction == pytest.approx(friction / 1.5, rel=1e-6)

  @pytest.mark.parametrize("platform", [CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021])
  def test_a_second_configure_does_not_compound(self, platform):
    # sunnypilot re-runs configure_torque_tune on the built CarParams when torque control is enforced
    from opendbc.car.mazda.interface import CarInterface
    CP = car_params(platform)
    before = (CP.lateralTuning.torque.latAccelFactor, CP.lateralTuning.torque.friction)
    CarInterface.configure_torque_tune(CP.carFingerprint, CP.lateralTuning)
    assert (CP.lateralTuning.torque.latAccelFactor, CP.lateralTuning.torque.friction) == pytest.approx(before, rel=1e-6)
