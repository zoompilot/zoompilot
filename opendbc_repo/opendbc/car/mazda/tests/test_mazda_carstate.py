"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

carstate through the real CarInterface and its parsers: the FSC settle gate, stockFcw, the
radar session response, GEAR.BRAKE_HOLD, the two-master guard, the speed sign unit, cancel
under braking, cruiseState.standstill and the LKAS non-delivery latch.
"""
import pytest

from opendbc.car import DT_CTRL
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.tests.conftest import car_interface, packer
from opendbc.car.mazda.values import CarControllerParams
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

CAM_LANEINFO = 0x440
CAM_EMPTY = 0x21d
CAM_PEDESTRIAN = 0x25d
CAM_TRAFFIC_SIGNS = 0x35f
GEAR = 0x228
RADAR_UDS_RESP = 0x76c

# Real CAM_LANEINFO prefixes, captured on two CX-5 2022s running the same FSC firmware
# (GSH7-67XK2-U). Only byte 1 differs: bit 5 is BIT2, bit 6 is NO_ERR_BIT.
BOOTING = bytes([0x42, 0b01000001, 0, 0, 0, 0, 0, 0])       # NO_ERR_BIT set: still booting
SETTLED = bytes([0x42, 0b00000001, 0, 0, 0, 0, 0, 0])       # markers clear: settled
BIT2_LATCHED = bytes([0x41, 0b00100001, 0, 0, 0, 0, 0, 0])  # BIT2 stuck high for a whole cycle
FAULTED = bytes([0x42, 0b00000001, 0, 0, 0, 0x01, 0, 0])    # ERR_BIT (bit 40) set

# Exercise CAM_LANEINFO at its longest measured period so freshness tests match the bus cadence.
CAM_LANEINFO_PERIOD_FRAMES = int(CarControllerParams.CAM_LANEINFO_PERIOD_T / DT_CTRL)

SETTLE_T = CarControllerParams.FSC_SETTLE_T
GUARD_T = CarControllerParams.STOCK_RADAR_GUARD_T


def t_ns(i):
  return int(i * DT_CTRL * 1e9)


def feed_laneinfo(CI, payload, secs):
  # payload None = a camera dropout, nothing on the bus at all
  n = int(secs / DT_CTRL)
  for i in range(n):
    msgs = [(CAM_LANEINFO, payload, 2)] if payload is not None and i % CAM_LANEINFO_PERIOD_FRAMES == 0 else []
    CI.update([(t_ns(i), msgs)])
  return CI.CS.fsc_settled


def feed(CI, i, *msgs):
  """One CarInterface.update with the packer tuples (or (addr, dat, bus) triples) given."""
  return CI.update([(t_ns(i), [(m[0], m[1], m[2]) for m in msgs])])


@pytest.mark.parametrize("alpha_long", [False, True])
def test_carstate_runs_with_real_parsers(alpha_long):
  # vl_all requires every accessed message to be registered by get_can_parsers.
  CI = car_interface(alpha_long)
  assert CI.CP.openpilotLongitudinalControl == alpha_long
  for _ in range(10):
    CI.update([])


class TestFscSettleGate:
  """The gate that defers the radar teardown past the FSC's cold-boot radar-presence check.

  It must hold while the camera is booting or faulted, and must not be vetoed indefinitely
  by a bit that carries no boot information.
  """

  def test_never_settles_while_boot_marker_is_set(self):
    assert not feed_laneinfo(car_interface(), BOOTING, SETTLE_T * 2)

  def test_never_settles_while_err_bit_is_set(self):
    # a latched i-ACTIVSENSE fault shows the boot markers clear, so ERR_BIT must veto on its own
    assert not feed_laneinfo(car_interface(), FAULTED, SETTLE_T * 2)

  def test_settles_once_the_boot_marker_clears(self):
    CI = car_interface()
    assert not feed_laneinfo(CI, BOOTING, 3.0)
    assert not feed_laneinfo(CI, SETTLED, SETTLE_T - 1.0)
    assert feed_laneinfo(CI, SETTLED, 1.5)

  def test_a_latched_bit2_does_not_block_the_teardown_forever(self):
    # BIT2 may remain high for an entire ignition cycle without indicating an incomplete boot.
    assert feed_laneinfo(car_interface(), BIT2_LATCHED, SETTLE_T * 1.5)

  def test_settles_at_the_longest_observed_camera_period(self):
    # The freshness window must span the longest measured CAM_LANEINFO period.
    assert feed_laneinfo(car_interface(), SETTLED, SETTLE_T * 1.5)

  def test_camera_dropout_resets_the_settle_timer(self):
    # A genuine camera dropout restarts the settle timer.
    CI = car_interface()
    feed_laneinfo(CI, SETTLED, SETTLE_T * 0.8)
    feed_laneinfo(CI, None, CarControllerParams.CAM_LANEINFO_FRESH_T + 0.5)
    assert not feed_laneinfo(CI, SETTLED, SETTLE_T * 0.5)
    assert feed_laneinfo(CI, SETTLED, SETTLE_T * 0.6)

  def test_gate_starts_closed_before_any_camera_frame(self):
    # the parser reads all-zero before the first frame, which would otherwise look settled
    CI = car_interface()
    for i in range(int(SETTLE_T * 2 / DT_CTRL)):
      CI.update([(t_ns(i), [])])
    assert not CI.CS.fsc_settled


class TestStockFcw:
  """0x21d (CAM_EMPTY) idles at STATUS 0x7f and leaves it only while the camera actively
  shows its SCBS collision display (route 0000004d t+213). The payloads are the captured
  idle and active frames from that route."""

  IDLE = bytes.fromhex("7f3fff00000affff")
  ACTIVE = bytes.fromhex("52124b00000ad294")

  def feed_21d(self, CI, payload, i=0):
    ret, _ = feed(CI, i, (CAM_EMPTY, payload, 2))
    return ret

  def test_display_active_sets_fcw(self):
    CI = car_interface()
    assert self.feed_21d(CI, self.IDLE).stockFcw is False
    assert self.feed_21d(CI, self.ACTIVE, 1).stockFcw is True
    assert self.feed_21d(CI, self.IDLE, 2).stockFcw is False

  def test_no_fcw_before_first_camera_frame(self):
    # the parser reads STATUS as 0 before the first frame, which is != 0x7f
    ret, _ = car_interface().update([(0, [])])
    assert ret.stockFcw is False

  def test_ped_warning_bit_sets_fcw(self):
    # never observed in 1.57M corpus frames, wired for coverage: PED_WARNING is bit 9
    CI = car_interface()
    self.feed_21d(CI, self.IDLE)
    ret, _ = feed(CI, 1, (CAM_PEDESTRIAN, bytes.fromhex("07fa3c0000000000"), 2), (CAM_EMPTY, self.IDLE, 2))
    assert ret.stockFcw is True


class TestRadarSessionResponse:
  """The radar answers session requests within ~10 ms (route 000000fe t+15.0), and the
  session manager consumes the flag on the same control frame it is set."""

  def test_negative_response_sets_refused(self):
    CI = car_interface()
    assert not CI.CS.radar_session_refused
    # 03 7F 10 22: conditionsNotCorrect to a session-control request
    feed(CI, 0, (RADAR_UDS_RESP, bytes.fromhex("037f102200000000"), 0))
    assert CI.CS.radar_session_refused
    feed(CI, 1)
    assert not CI.CS.radar_session_refused, "the flag is same-frame, not latched"

  def test_positive_response_is_not_a_refusal(self):
    CI = car_interface()
    # the real capture: 06 50 02 with the session parameter record (P2*=5.0 s)
    feed(CI, 0, (RADAR_UDS_RESP, bytes.fromhex("065002001901f400"), 0))
    assert not CI.CS.radar_session_refused

  def test_response_pending_is_not_a_refusal(self):
    CI = car_interface()
    # 03 7F 10 78: requestCorrectlyReceived-ResponsePending; UDS clients wait through it
    feed(CI, 0, (RADAR_UDS_RESP, bytes.fromhex("037f107800000000"), 0))
    assert not CI.CS.radar_session_refused


class TestBrakeHold:
  """GEAR.BRAKE_HOLD is the body ECU reporting that it owns the standstill hold. Stock relaxes
  its own command the instant this sets, so the payloads below come straight off the two logs
  that pinned the signal down: a hold that latched (route caace206f6 seg 8, 0x17 at 1157.34 s)
  and one that never did (route 00000065 seg 4, stuck at 0x07 while the car crept)."""

  @pytest.mark.parametrize("payload, expected", [
    ("142007ff02f00000", False),  # hold not taken over: keep braking
    ("142017ff02f00000", True),   # body has the brakes
    ("14200fff02f00000", False),  # released again at the resume
  ])
  def test_decodes_the_hold_bit(self, payload, expected):
    CI = car_interface()
    # CANParser registers a message lazily on first access, so the first frame only arms it
    for i in range(2):
      feed(CI, i, (GEAR, bytes.fromhex(payload), 0))
    assert CI.CS.brake_hold is expected

  def test_defaults_to_not_held(self):
    # nothing parsed yet must read as "the car is not holding", the direction that keeps braking
    assert not car_interface().CS.brake_hold


def feed_guard(CI, secs, radar_alive, start_frame=0, acc_active=False):
  pk = packer()
  ret = None
  n = int(secs / DT_CTRL)
  for i in range(start_frame, start_frame + n):
    msgs = [pk.make_can_msg("PEDALS", 0, {"ACC_OFF": 0 if acc_active else 1, "ACC_ACTIVE": 1 if acc_active else 0})]
    if radar_alive:
      msgs.append(mazdacan.create_acc_command(pk, 0, i, 0., long_active=False, acc_available=True))
    ret, _ = feed(CI, i, *msgs)
  return ret, start_frame + n


class TestTwoMasterGuard:
  """The stock-radar guard wears two hats: before the first teardown it is the expected boot
  phase and must only hold availability low (no fault alert); once the radar has been silenced,
  hearing it again is a genuine two-master conflict and must raise accFaulted."""

  def test_boot_phase_is_not_a_fault(self):
    # radar broadcasting, teardown not started: engagement blocked quietly, no Cruise Fault
    ret, _ = feed_guard(car_interface(), 5.0, radar_alive=True)
    assert not ret.accFaulted
    assert not ret.cruiseState.available

  def test_availability_arrives_with_radar_silence(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, start_frame=n)
    assert not ret.accFaulted
    assert ret.cruiseState.available

  def test_availability_trails_the_pandas_radar_latch(self):
    # carstate availability must follow panda's matching radar-ownership guard.
    panda_latch = (CarControllerParams.STOCK_RADAR_ALIVE_T + CarControllerParams.LONG_STEP * DT_CTRL +
                   CarControllerParams.PANDA_RADAR_SILENT_T)
    assert panda_latch < GUARD_T
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    ret, n = feed_guard(CI, panda_latch + 0.05, radar_alive=False, start_frame=n)
    assert not ret.cruiseState.available
    assert not CI.CS.stock_radar_alive
    assert not CI.CS.stock_radar_gone
    ret, n = feed_guard(CI, GUARD_T - panda_latch, radar_alive=False, start_frame=n)
    assert ret.cruiseState.available
    assert CI.CS.stock_radar_gone

  def test_radar_return_after_teardown_is_a_fault(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, start_frame=n)
    ret, n = feed_guard(CI, 0.5, radar_alive=True, start_frame=n)
    assert ret.accFaulted
    # A transient radar return reports a fault without revoking latched availability.
    assert ret.cruiseState.available
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, start_frame=n)
    assert not ret.accFaulted
    assert ret.cruiseState.available

  def test_stock_engagement_inside_the_guard_is_not_reported(self):
    # Do not expose stock MRCC engagement before radar ownership transfers.
    ret, _ = feed_guard(car_interface(), 5.0, radar_alive=True, acc_active=True)
    assert not ret.cruiseState.available
    assert not ret.cruiseState.enabled

  def test_engagement_still_live_when_the_guard_lifts_is_not_adopted(self):
    # Silence alone must not turn a pre-existing stock engagement into an openpilot engage:
    # that edge would arrive with no driver input behind it.
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True, acc_active=True)
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, acc_active=True, start_frame=n)
    assert ret.cruiseState.available
    assert not ret.cruiseState.enabled

  def test_engagement_after_an_idle_sample_is_adopted(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True, acc_active=True)
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, acc_active=True, start_frame=n)
    ret, n = feed_guard(CI, 0.2, radar_alive=False, acc_active=False, start_frame=n)
    assert not ret.cruiseState.enabled
    ret, n = feed_guard(CI, 0.2, radar_alive=False, acc_active=True, start_frame=n)
    assert ret.cruiseState.enabled


class TestSpeedSignLimit:
  """CAM_TRAFFIC_SIGNS.SPEED_SIGN_UNIT is the 2-bit field carrying the display unit (upstream's
  1-bit SPEED_SIGN_ON at bit 12 is its low bit): 1 = limit displayed in mph, 2 = displayed in
  km/h, 0 = none. Which value an FSC emits tracks its market, not the cluster's unit setting.
  Payloads are real captures: mph frames from a US CX-5 2022 (drive_1x local set), km/h
  frames from a NZ CX-5 (route
  ded445e51c0e1830|00000007--4b5a89a1ce) where the old 1-bit decode at bit 12 read 0 and SLA
  never saw a limit."""

  @pytest.mark.parametrize("payload, expected_ms", [
    ("0000000002005300", 0.0),                 # no limit displayed
    ("0650000002005300", 25 * CV.MPH_TO_MS),   # US 25 mph
    ("0a10000003001300", 40 * CV.MPH_TO_MS),   # US 40 mph
    ("0b50000002005300", 45 * CV.MPH_TO_MS),   # US 45 mph
    ("0a20000002000900", 40 * CV.KPH_TO_MS),   # NZ 40 km/h
    ("0ca0000002000900", 50 * CV.KPH_TO_MS),   # NZ 50 km/h
    ("1920000002010900", 100 * CV.KPH_TO_MS),  # NZ 100 km/h
  ])
  def test_unit_comes_from_the_frame(self, payload, expected_ms):
    CI = car_interface()
    ret_sp = None
    for i in range(2):
      _, ret_sp = feed(CI, i, (CAM_TRAFFIC_SIGNS, bytes.fromhex(payload), 2))
    assert ret_sp.speedLimit == pytest.approx(expected_ms, rel=1e-6, abs=1e-12)

  @pytest.mark.parametrize("sign_on, speed_sign", [
    (1, 120),  # above any real mph posting
    (2, 127),  # all-ones: invalid sentinel
    (3, 50),   # undefined state
    (1, 0),    # displayed-but-zero
  ])
  def test_implausible_frames_read_as_no_limit(self, sign_on, speed_sign):
    msg = packer().make_can_msg("CAM_TRAFFIC_SIGNS", 2, {"SPEED_SIGN_UNIT": sign_on, "SPEED_SIGN": speed_sign})
    CI = car_interface()
    ret_sp = None
    for i in range(2):
      _, ret_sp = feed(CI, i, msg)
    assert ret_sp.speedLimit == 0.0


class TestCancelUnderBraking:
  """The availability brake-hold exists for brake-only PEDALS samples that arrive with both
  bits low mid-press. A wheel CANCEL turns the MRCC main state off for real and must land
  even with the brake down (route 7f9e3ff336 t+484-488: cancel mashed under braking was
  swallowed until the brake released 4 s later)."""

  @staticmethod
  def armed_and_silent(CI):
    # get past the two-master guard with the main armed so availability starts True
    pk = packer()
    n = int((GUARD_T + 0.5) / DT_CTRL)
    for i in range(n):
      ret, _ = feed(CI, i, pk.make_can_msg("PEDALS", 0, {"ACC_OFF": 1}))
    assert ret.cruiseState.available
    return pk, n

  @staticmethod
  def feed_pedals(CI, pk, n0, secs, brake, cancel):
    ret = None
    n = int(secs / DT_CTRL)
    for i in range(n0, n0 + n):
      ret, _ = feed(CI, i, pk.make_can_msg("PEDALS", 0, {"ACC_OFF": 0, "BRAKE_ON": int(brake)}),
                    pk.make_can_msg("CRZ_BTNS", 0, {"CAN_OFF": int(cancel)}))
    return ret, n0 + n

  def test_brake_only_dropout_is_held(self):
    CI = car_interface()
    pk, n = self.armed_and_silent(CI)
    ret, n = self.feed_pedals(CI, pk, n, 1.0, brake=True, cancel=False)
    assert ret.cruiseState.available

  def test_cancel_lands_through_the_brake(self):
    CI = car_interface()
    pk, n = self.armed_and_silent(CI)
    ret, n = self.feed_pedals(CI, pk, n, 0.3, brake=True, cancel=True)
    assert not ret.cruiseState.available

  def test_cancel_context_outlives_the_press(self):
    # the PEDALS reaction can trail the button: press-and-release while still armed, then the
    # bits drop only after the button is back up -- the context memory has to carry it
    CI = car_interface()
    pk, n = self.armed_and_silent(CI)
    ret = None
    for i in range(n, n + 5):  # cancel pressed, PEDALS not yet reacting
      ret, _ = feed(CI, i, pk.make_can_msg("PEDALS", 0, {"ACC_OFF": 1}), pk.make_can_msg("CRZ_BTNS", 0, {"CAN_OFF": 1}))
    assert ret.cruiseState.available
    ret, n = self.feed_pedals(CI, pk, n + 5, 0.2, brake=True, cancel=False)
    assert not ret.cruiseState.available


class TestCruiseStandstill:
  """PEDALS.STANDSTILL is the PCM's wheel-speed "stopped" bit, not a stock-ACC hold state.

  LongControl's starting_condition is `not should_stop and not cruise_standstill and not
  brake_pressed`, so reporting it under openpilot longitudinal deadlocks every stop: long
  control holds LongCtrlState.stopping (and with it stopAccel) until the car moves, and the
  car cannot move until long control leaves stopping. Both engaged stops on route
  000000fa--6b21bd7e7e (2026-08-25) sat pinned at -1.03 m/s2 through a departing lead, with
  the plan asking for +1.3, until the driver used the gas pedal. Nothing downstream ever ran:
  no RESUME_UNLATCHING pulse and no RES press, since both key off actuators.accel > 0.
  """

  @pytest.mark.parametrize("alpha_long, expected", [
    # the stock MRCC is not in the loop at all under openpilot longitudinal: its radar is
    # silenced and we synthesize its frames, so there is no stock standstill state to report
    (True, False),
    # stock long still needs it: it is what drives CC.cruiseControl.resume in controlsd
    (False, True),
  ], ids=["alpha_long", "stock_long"])
  def test_reported_only_with_stock_longitudinal(self, alpha_long, expected):
    pk = packer()
    CI = car_interface(alpha_long)
    ret = None
    for i in range(2):  # CANParser registers a message lazily, so the first frame only arms it
      ret, _ = feed(CI, i, pk.make_can_msg("PEDALS", 0, {"STANDSTILL": 1}))
    assert ret.cruiseState.standstill is expected


class UndeliveredRig:
  """A stock-long CX-5 2022 fed STEER_RATE / WHEEL_SPEEDS / STEER_TORQUE frame by frame."""

  def __init__(self):
    self.CI = car_interface(alpha_long=False)
    self.CS = self.CI.CS
    self.params = self.CS.params
    self.packer = packer()
    self.frame = 0

  def step(self, request, effective, blocked, speed_kph=40., driver_torque=0, track_state=0):
    self.frame += 1
    ret, _ = feed(self.CI, self.frame,
                  self.packer.make_can_msg("STEER_RATE", 0, {"LKAS_REQUEST": request, "LKAS_EFFECTIVE": effective, "LKAS_BLOCK": blocked,
                                                             "LKAS_TRACK_STATE": track_state}),
                  self.packer.make_can_msg("WHEEL_SPEEDS", 0, {"FL": speed_kph, "FR": speed_kph, "RL": speed_kph, "RR": speed_kph}),
                  self.packer.make_can_msg("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": driver_torque}))
    return ret


class TestSteerUndeliveredLatch:
  """The LKAS non-delivery latch lives in carstate, fed only by the EPS's STEER_RATE report."""

  def test_rejected_request_latches_then_alerts(self):
    rig = UndeliveredRig()
    for _ in range(100):
      rig.step(600, 600, 0)
    assert not rig.CS.steer_undelivered
    # Latch after the configured number of zero-delivery frames.
    for _ in range(rig.params.STEER_UNDELIVERED_FRAMES - 1):
      ret = rig.step(600, 0, 1)
    assert not rig.CS.steer_undelivered
    ret = rig.step(600, 0, 1)
    assert rig.CS.steer_undelivered
    assert not ret.steerFaultTemporary
    # Alert only after the additional configured hold time.
    for _ in range(rig.params.STEER_UNDELIVERED_ALERT_FRAMES - 2):
      ret = rig.step(0, 0, 1)
    assert not ret.steerFaultTemporary
    ret = rig.step(0, 0, 1)
    assert ret.steerFaultTemporary
    ret = rig.step(600, 600, 0)
    assert not rig.CS.steer_undelivered
    assert not ret.steerFaultTemporary

  def test_small_or_delivered_requests_never_latch(self):
    rig = UndeliveredRig()
    for _ in range(200):
      rig.step(rig.params.STEER_UNDELIVERED_MIN, 0, 1)  # not above the minimum
    assert not rig.CS.steer_undelivered
    for _ in range(200):
      rig.step(600, 150, 1)  # blocked but still delivering something
    assert not rig.CS.steer_undelivered

  def test_latch_below_manoeuvring_speed_stays_silent(self):
    rig = UndeliveredRig()
    for _ in range(rig.params.STEER_UNDELIVERED_FRAMES + rig.params.STEER_UNDELIVERED_ALERT_FRAMES + 50):
      ret = rig.step(600, 0, 1, speed_kph=10.)
    assert rig.CS.steer_undelivered
    assert not ret.steerFaultTemporary

  def test_launch_block_stays_silent_however_fast_it_goes(self):
    # LKAS_TRACK_STATE identifies normal standby blocks that can persist into a brisk launch.
    rig = UndeliveredRig()
    hold = rig.params.STEER_UNDELIVERED_FRAMES + rig.params.STEER_UNDELIVERED_ALERT_FRAMES + 50
    for _ in range(hold):
      ret = rig.step(600, 0, 1, speed_kph=25., track_state=1)
    assert rig.CS.steer_undelivered  # the command is still zeroed, only the banner is withheld
    assert not ret.steerFaultTemporary
    # Clearing standby while still blocked makes the condition alertable.
    ret = rig.step(0, 0, 1, speed_kph=25., track_state=0)
    assert ret.steerFaultTemporary

  def test_driver_steering_with_the_request_still_latches(self):
    # Driver torque in the requested direction does not reduce the command and must not gate
    # non-delivery detection.
    rig = UndeliveredRig()
    for _ in range(rig.params.STEER_UNDELIVERED_FRAMES + 5):
      rig.step(600, 0, 1, driver_torque=20)
    assert rig.CS.steer_undelivered

  @pytest.mark.parametrize("v", [0.0, 1.0, "just_below_alert_speed"])
  def test_no_alert_at_or_near_standstill(self, v):
    # Zero delivery is normal at maneuvering speed, although the command remains withheld.
    rig = UndeliveredRig()
    if v == "just_below_alert_speed":
      v = rig.params.STEER_UNDELIVERED_ALERT_MIN_SPEED - 0.5
    for _ in range(400):
      ret = rig.step(600, 0, 1, speed_kph=v * CV.MS_TO_KPH)
    assert not ret.steerFaultTemporary, f"alerted at {v} m/s"
    assert rig.CS.steer_undelivered, f"did not latch at {v} m/s"

  def test_accelerating_out_of_a_block_that_began_rolling_does_alert(self):
    # A persistent block that began above the origin speed becomes alertable when vehicle
    # speed crosses the threshold.
    rig = UndeliveredRig()
    v0 = rig.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED + 1.0
    for _ in range(300):
      ret = rig.step(600, 0, 1, speed_kph=v0 * CV.MS_TO_KPH)
    assert not ret.steerFaultTemporary
    for _ in range(5):
      ret = rig.step(600, 0, 1, speed_kph=7.0 * CV.MS_TO_KPH)
    assert ret.steerFaultTemporary

  def test_block_carried_from_a_stop_never_alerts_however_long_it_lasts(self):
    # The EPS's standby from a stop can outlive TRACK_STATE through a slow crawl (9 blocks
    # of 13 to 44 s in 64 h of drives, none a fault). The origin speed, not the state bit,
    # separates them from a dropout; the command stays zeroed either way.
    rig = UndeliveredRig()
    for _ in range(50):
      rig.step(600, 0, 1, speed_kph=0.0, track_state=1)
    for _ in range(600):
      ret = rig.step(600, 0, 1, speed_kph=7.0 * CV.MS_TO_KPH, track_state=0)
    assert rig.CS.steer_undelivered
    assert not ret.steerFaultTemporary
    # A release resets the origin, so the next block is judged on its own start.
    rig.step(600, 600, 0, speed_kph=7.0 * CV.MS_TO_KPH)
    hold = rig.params.STEER_UNDELIVERED_FRAMES + rig.params.STEER_UNDELIVERED_ALERT_FRAMES + 5
    for _ in range(hold):
      ret = rig.step(600, 0, 1, speed_kph=7.0 * CV.MS_TO_KPH, track_state=0)
    assert ret.steerFaultTemporary

  def test_origin_is_the_speed_at_the_blocks_first_frame(self):
    # Creeping when the block begins reads as a stop; rolling reads as a dropout.
    for v0, expect in ((0.3, False), (2.0, True)):
      rig = UndeliveredRig()
      for _ in range(2):  # the parser applies a frame on the update after it arrives
        rig.step(600, 0, 1, speed_kph=v0 * CV.MS_TO_KPH, track_state=0)
      for _ in range(rig.params.STEER_UNDELIVERED_FRAMES + rig.params.STEER_UNDELIVERED_ALERT_FRAMES + 5):
        ret = rig.step(600, 0, 1, speed_kph=7.0 * CV.MS_TO_KPH, track_state=0)
      assert ret.steerFaultTemporary == expect, f"origin {v0} m/s"


class TestTjaButtonEvents:
  """The physical TJA button is published as an lkas event alongside, not instead of,
  mainCruise, once the driver has declared the button. Undeclared, bit 11 is ignored so a
  stray press can never toggle lateral on a car that runs the ACC-main path."""

  ButtonType = structs.CarState.ButtonEvent.Type

  def _btns(self, CI, pk, i, **values):
    ret, _ = feed(CI, i, pk.make_can_msg("CRZ_BTNS", 0, values))
    return ret

  def _declared(self):
    CI = car_interface(alpha_long=False)
    CI.CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON
    return CI

  def test_tja_press_emits_an_lkas_event(self):
    CI, pk = self._declared(), packer()
    self._btns(CI, pk, 0, TJA_BUTTON=0)
    ret = self._btns(CI, pk, 1, TJA_BUTTON=1)
    assert [be.type for be in ret.buttonEvents] == [self.ButtonType.lkas]
    assert ret.buttonEvents[0].pressed

  def test_no_event_without_the_button(self):
    CI, pk = self._declared(), packer()
    for i in range(10):
      ret = self._btns(CI, pk, i, TJA_BUTTON=0)
      assert not [be for be in ret.buttonEvents if be.type == self.ButtonType.lkas]

  def test_undeclared_ignores_the_bit(self):
    CI, pk = car_interface(alpha_long=False), packer()
    self._btns(CI, pk, 0, TJA_BUTTON=0)
    ret = self._btns(CI, pk, 1, TJA_BUTTON=1)
    assert not [be for be in ret.buttonEvents if be.type == self.ButtonType.lkas]

  def test_main_cruise_event_is_unchanged(self):
    CI, pk = car_interface(alpha_long=False), packer()
    self._btns(CI, pk, 0, MODE_X=0, MODE_Y=0)
    ret = self._btns(CI, pk, 1, MODE_X=1, MODE_Y=1)
    assert [be.type for be in ret.buttonEvents] == [self.ButtonType.mainCruise]
