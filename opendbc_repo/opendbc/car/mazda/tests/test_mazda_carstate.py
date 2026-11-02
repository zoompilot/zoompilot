"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

carstate through the real CarInterface and its parsers: the FSC settle gate, stockFcw, the
radar session response, the body hold, the two-master guard, the speed sign unit, cancel
under braking, cruiseState.standstill and the LKAS non-delivery latch.
"""
import pytest

from opendbc.car import Bus, DT_CTRL
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carstate import ButtonType, LKAS_REARM_FRAMES, LKAS_REARM_FAULT_FRAMES
from opendbc.car.mazda.tests.conftest import car_interface, car_params, car_params_sp, packer
from opendbc.car.mazda.values import CAR, CarControllerParams

MAIN_OFF_DEBOUNCE_SAMPLES = CarControllerParams.MAIN_OFF_DEBOUNCE_SAMPLES
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

CAM_LANEINFO = 0x440
CAM_LKAS = 0x243
CAM_EMPTY = 0x21d
CAM_PEDESTRIAN = 0x25d
CAM_TRAFFIC_SIGNS = 0x35f
NAV_SPEED_LIMIT = 0x3d1
GEAR = 0x228
EPB = 0x79
RADAR_UDS_RESP = 0x76c

# Real CAM_LANEINFO prefixes, captured on two CX-5 2022s running the same FSC firmware
# (GSH7-67XK2-U). Only byte 1 differs: bit 5 is BIT2, bit 6 is NO_ERR_BIT.
BOOTING = bytes([0x42, 0b01000001, 0, 0, 0, 0, 0, 0])       # NO_ERR_BIT set: still booting
SETTLED = bytes([0x42, 0b00000001, 0, 0, 0, 0, 0, 0])       # markers clear: settled
BIT2_LATCHED = bytes([0x41, 0b00100001, 0, 0, 0, 0, 0, 0])  # BIT2 (HBC armed) high for a whole cycle
FAULTED = bytes([0x42, 0b00000001, 0, 0, 0, 0x01, 0, 0])    # ERR_BIT (bit 40) set

# Exercise CAM_LANEINFO at its longest measured period, 0.563 s, so freshness tests match the bus cadence.
CAM_LANEINFO_PERIOD_FRAMES = int(0.563 / DT_CTRL)

SETTLE_T = CarControllerParams.FSC_SETTLE_T
GUARD_T = CarControllerParams.STOCK_RADAR_GUARD_T
ACK_GUARD_T = CarControllerParams.STOCK_RADAR_ACK_GUARD_T
PROGRAMMING_ACK = bytes.fromhex("065002001901f400")  # route 00000025 t+10.538


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
  """The gate that holds the radar teardown until the camera (FSC) has booted.

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

  def test_hbc_arming_follows_bit2_while_fresh(self):
    # BIT2 is the camera's auto high-beam arming, relayed to the cluster through CRZ_CTRL
    CI = car_interface(alpha_long=True)
    feed_laneinfo(CI, BIT2_LATCHED, 1.0)
    assert CI.CS.hbc_request
    feed_laneinfo(CI, SETTLED, 1.0)
    assert not CI.CS.hbc_request
    # a silent camera relays nothing
    feed_laneinfo(CI, BIT2_LATCHED, 1.0)
    feed_laneinfo(CI, None, CarControllerParams.CAM_LANEINFO_FRESH_T + 0.5)
    assert not CI.CS.hbc_request

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


class TestBodyHold:
  """The body ECU reporting that it owns the standstill hold. EPB.HOLD_STATE is the handshake stock
  MRCC relaxes on, with or without Auto Hold; GEAR.BRAKE_HOLD joins it only with Auto Hold armed.
  Payloads are real: device_data 0000012b seg 5 (stock MRCC holding at t+311.3 with GEAR still at
  0x07), route 118 seg 9 (alpha long, both set at t+560.0) and the 2016.5 CX-5 KE, whose body has
  no standstill hold."""

  @pytest.mark.parametrize("epb, gear, expected", [
    ("df5c3206005f4d4f", "142007ff02f00000", False),  # idle: keep braking
    ("df463306011f4c58", "142007ff02f00000", True),   # body holds, Auto Hold not in it
    ("df5c3306009f4952", "142017ff02f00000", True),   # body holds, Auto Hold armed
    ("df523506005f445d", "142007ff02f00000", False),  # releasing at the resume
    ("df5a350600df4655", "14200fff02f00000", False),  # releasing, Auto Hold dropping out
    ("df5c3206005f4d4f", "142017ff02f00000", True),   # Auto Hold alone still holds the car
    ("dddc0001bd9e0e21", "242807ff04f00000", False),  # KE: no hold to hand over, keep braking
  ])
  def test_decodes_the_hold(self, epb, gear, expected):
    CI = car_interface()
    # CANParser registers GEAR lazily on first access, so the first frame only arms it
    for i in range(2):
      feed(CI, i, (EPB, bytes.fromhex(epb), 0), (GEAR, bytes.fromhex(gear), 0))
    assert CI.CS.body_hold is expected

  def test_defaults_to_not_held(self):
    # nothing parsed yet must read as "the car is not holding", the direction that keeps braking
    assert not car_interface().CS.body_hold

  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_a_body_without_the_frame_keeps_can_valid(self, alpha_long):
    pt = car_interface(alpha_long=alpha_long).can_parsers[Bus.pt]
    assert pt.message_states[EPB].ignore_alive


def feed_guard(CI, secs, radar_alive, start_frame=0, acc_active=False):
  pk = packer()
  ret = None
  n = int(secs / DT_CTRL)
  CI.CC.radar_session.control_active = not radar_alive  # the controller's claim, from its session in production
  for i in range(start_frame, start_frame + n):
    msgs = [pk.make_can_msg("PEDALS", 0, {"ACC_OFF": 0 if acc_active else 1, "ACC_ACTIVE": 1 if acc_active else 0})]
    msgs.append(pk.make_can_msg("ENGINE_DATA", 0, {"SPEED": 0}))
    if radar_alive:
      msgs.append(mazdacan.create_acc_command(pk, 0, i, 0., long_active=False, acc_available=True))
    ret, _ = feed(CI, i, *msgs)
  return ret, start_frame + n


class TestTwoMasterGuard:
  """Availability is the main switch from the first frame (lateral needs only that, and the
  panda reads the same PEDALS sample). The radar guard gates cruise: before the first teardown
  it is the expected boot phase and must only hold enabled low (no fault alert); once the radar
  has been silenced, hearing it again is a genuine two-master conflict and must raise accFaulted."""

  def test_boot_phase_is_not_a_fault(self):
    # radar broadcasting, teardown not started: main on shows (MADS may arm), cruise is not
    # owned, no Cruise Fault
    CI = car_interface()
    ret, _ = feed_guard(CI, 5.0, radar_alive=True)
    assert not ret.accFaulted
    assert ret.cruiseState.available
    assert not ret.cruiseState.enabled
    assert not CI.CS.radar_owned

  def test_ownership_arrives_with_radar_silence(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, start_frame=n)
    assert not ret.accFaulted
    assert CI.CS.radar_owned
    assert ret.cruiseState.available

  def test_ownership_needs_the_whole_guard(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    ret, n = feed_guard(CI, GUARD_T - 0.1, radar_alive=False, start_frame=n)
    assert not CI.CS.radar_owned
    assert not CI.CS.stock_radar_gone
    assert ret.cruiseState.available  # the main switch never waited
    ret, n = feed_guard(CI, 0.2, radar_alive=False, start_frame=n)
    assert CI.CS.radar_owned
    assert CI.CS.stock_radar_gone

  def ack(self, CI, n, requested=True):
    # the radar's positive reply to our programming request, on a healthy bus
    CI.CC.radar_session.programming_sent = requested
    pk = packer()
    feed(CI, n, pk.make_can_msg("PEDALS", 0, {"ACC_OFF": 1}), pk.make_can_msg("ENGINE_DATA", 0, {"SPEED": 0}),
         (RADAR_UDS_RESP, PROGRAMMING_ACK, 0))
    return n + 1

  def test_an_acknowledged_request_shortens_the_guard(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    n = self.ack(CI, n)
    ret, n = feed_guard(CI, ACK_GUARD_T - 0.05, radar_alive=False, start_frame=n)
    assert not CI.CS.radar_owned
    ret, n = feed_guard(CI, 0.1, radar_alive=False, start_frame=n)
    assert CI.CS.radar_owned

  def test_stock_traffic_after_the_ack_needs_the_whole_guard(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    n = self.ack(CI, n)
    ret, n = feed_guard(CI, 0.1, radar_alive=True, start_frame=n)
    ret, n = feed_guard(CI, ACK_GUARD_T + 0.1, radar_alive=False, start_frame=n)
    assert not CI.CS.radar_owned
    ret, n = feed_guard(CI, GUARD_T, radar_alive=False, start_frame=n)
    assert CI.CS.radar_owned

  def test_an_ack_without_our_request_counts_for_nothing(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    n = self.ack(CI, n, requested=False)
    ret, n = feed_guard(CI, ACK_GUARD_T + 0.1, radar_alive=False, start_frame=n)
    assert not CI.CS.radar_owned

  def test_radar_return_after_teardown_is_a_fault(self):
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, start_frame=n)
    ret, n = feed_guard(CI, 0.5, radar_alive=True, start_frame=n)
    assert ret.accFaulted
    # A returned radar revokes ownership (cruise), not the main switch: lateral stays.
    assert not CI.CS.radar_owned
    assert ret.cruiseState.available
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, start_frame=n)
    assert not ret.accFaulted
    assert CI.CS.radar_owned

  def test_a_bus_blip_does_not_rerun_the_guard(self):
    # the radar is in its diagnostic session whatever the vehicle bus does: a witness gap the
    # CANParser would also see as invalid (canValid blocks engagement on its own) neither
    # revokes ownership nor re-runs the 1.27 s guard once the bus is back
    CI = car_interface()
    ret, n = feed_guard(CI, 5.0, radar_alive=True)
    ret, n = feed_guard(CI, GUARD_T + 0.5, radar_alive=False, start_frame=n)
    assert ret.cruiseState.available
    for i in range(n, n + 30):  # 0.3 s of nothing at all, past both witness windows
      ret, _ = feed(CI, i)
    n += 30
    assert not CI.CS.radar_bus_healthy
    assert not CI.CS.stock_radar_gone  # the silence is the bus, not evidence
    assert ret.cruiseState.available
    assert not ret.accFaulted
    ret, n = feed_guard(CI, 0.05, radar_alive=False, start_frame=n)
    assert CI.CS.radar_bus_healthy
    assert ret.cruiseState.available

  def test_a_dead_bus_is_not_adopted_as_a_silenced_radar(self):
    # boot with no vehicle traffic at all: the silence is the bus, not a teardown
    CI = car_interface()
    ret = None
    for i in range(int((GUARD_T + 1.0) / DT_CTRL)):
      ret, _ = feed(CI, i)
    assert not CI.CS.stock_radar_gone
    assert not ret.cruiseState.available

  def test_stock_engagement_inside_the_guard_is_not_reported(self):
    # A stock MRCC engagement before the takeover is the body's: main shows, enabled does not.
    ret, _ = feed_guard(car_interface(), 5.0, radar_alive=True, acc_active=True)
    assert ret.cruiseState.available
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


def speed_limit(*msgs):
  """carStateSP.speedLimit after two frames of the given messages."""
  CI = car_interface()
  ret_sp = None
  for i in range(2):
    _, ret_sp = feed(CI, i, *msgs)
  return ret_sp.speedLimit


class TestSpeedSignLimit:
  """CAM_TRAFFIC_SIGNS.SPEED_SIGN_UNIT is the 2-bit field carrying the display unit (upstream's
  1-bit SPEED_SIGN_ON at bit 12 is its low bit): 1 = limit displayed in mph, 2 = displayed in
  km/h, 0 = none. Which value an FSC emits tracks its market, not the cluster's unit setting.
  Payloads are real captures: mph frames from a US CX-5 2022 (drive_1x local set), km/h
  frames from a NZ CX-5, where the old 1-bit decode at bit 12 read 0 and SLA never saw a limit."""

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
    assert speed_limit((CAM_TRAFFIC_SIGNS, bytes.fromhex(payload), 2)) == pytest.approx(expected_ms, rel=1e-6, abs=1e-12)

  @pytest.mark.parametrize("sign_on, speed_sign", [
    (1, 120),  # above any real mph posting
    (2, 127),  # all-ones: invalid sentinel
    (3, 50),   # undefined state
    (1, 0),    # displayed-but-zero
  ])
  def test_implausible_frames_read_as_no_limit(self, sign_on, speed_sign):
    assert speed_limit(packer().make_can_msg("CAM_TRAFFIC_SIGNS", 2, {"SPEED_SIGN_UNIT": sign_on, "SPEED_SIGN": speed_sign})) == 0.0


class TestNavSpeedLimit:
  """NAV_SPEED_LIMIT (0x3D1, car bus) is the navigation unit's map limit, in the same 7-bit value
  plus 2-bit unit layout as CAM_TRAFFIC_SIGNS, at bytes 3-4. A 2021 CX-5 without camera sign
  recognition sends CAM_TRAFFIC_SIGNS all zero and the limit only here. On a CX-5 2022 with both,
  the two agree on 81% of frames and the camera's reading of a posted sign wins the rest.
  Payloads are real US captures (routes 0000026e-0000027f); the km/h case is packed, by analogy
  with CAM_TRAFFIC_SIGNS, as no metric nav capture exists yet."""

  @pytest.mark.parametrize("nav, cam, expected_ms", [
    ("000000000011091a", "0000000002005300", 0.0),                # no limit on either
    ("000000065011051a", "0000000002005300", 25 * CV.MPH_TO_MS),  # blank camera: the map fills in
    ("0000000b5011091a", "0000000002005300", 45 * CV.MPH_TO_MS),
    ("000000065011051a", None, 25 * CV.MPH_TO_MS),                # no camera frame at all
    ("000000065011051a", "0b50000002005300", 45 * CV.MPH_TO_MS),  # the camera's sign beats the map
  ])
  def test_camera_first_then_nav(self, nav, cam, expected_ms):
    msgs = [(NAV_SPEED_LIMIT, bytes.fromhex(nav), 0)]
    if cam is not None:
      msgs.append((CAM_TRAFFIC_SIGNS, bytes.fromhex(cam), 2))
    assert speed_limit(*msgs) == pytest.approx(expected_ms, rel=1e-6, abs=1e-12)

  def test_metric_nav_limit(self):
    assert speed_limit(packer().make_can_msg("NAV_SPEED_LIMIT", 0, {"SPEED_SIGN_UNIT": 2, "SPEED_SIGN": 60})) == pytest.approx(60 * CV.KPH_TO_MS)


class TestMainOffDebounce:
  """Under alpha long, main is PEDALS arming, debounced off: both bits low for
  MAIN_OFF_DEBOUNCE_SAMPLES samples is a main-off, brake or no brake. It replaces a brake-hold that
  let a both-low run under braking land only inside a button's context, which missed a cancel
  mashed under braking (route 7f9e3ff336 t+484-488), the KE's MODE_X main-off at a stop
  (route_ke_0b seg 3), and a main-off with no visible button (000001f0--b507914e9e seg 0 t+10.69).
  No both-low run that came back on its own appears in 4026 segments."""

  @staticmethod
  def armed_and_silent(CI):
    # get past the two-master guard with the main armed so availability starts True
    pk = packer()
    CI.CC.radar_session.control_active = True
    n = int((GUARD_T + 0.5) / DT_CTRL)
    for i in range(n):
      ret, _ = feed(CI, i, pk.make_can_msg("PEDALS", 0, {"ACC_OFF": 1}), pk.make_can_msg("ENGINE_DATA", 0, {"SPEED": 0}))
    assert ret.cruiseState.available
    return pk, n

  @staticmethod
  def feed_pedals(CI, pk, n0, samples, brake, armed=False):
    # one PEDALS sample per feed
    ret = None
    for i in range(n0, n0 + samples):
      ret, _ = feed(CI, i, pk.make_can_msg("ENGINE_DATA", 0, {"SPEED": 0}), pk.make_can_msg("PEDALS", 0, {"ACC_OFF": int(armed), "BRAKE_ON": int(brake)}))
    return ret, n0 + samples

  @pytest.mark.parametrize("brake", [True, False], ids=["brake", "no_brake"])
  def test_main_off_lands_after_the_debounce(self, brake):
    CI = car_interface()
    pk, n = self.armed_and_silent(CI)
    ret, n = self.feed_pedals(CI, pk, n, MAIN_OFF_DEBOUNCE_SAMPLES - 1, brake)
    assert ret.cruiseState.available
    ret, n = self.feed_pedals(CI, pk, n, 1, brake)
    assert not ret.cruiseState.available

  def test_short_dropout_is_held(self):
    CI = car_interface()
    pk, n = self.armed_and_silent(CI)
    for _ in range(3):
      ret, n = self.feed_pedals(CI, pk, n, MAIN_OFF_DEBOUNCE_SAMPLES - 1, brake=True)
      assert ret.cruiseState.available
      ret, n = self.feed_pedals(CI, pk, n, 1, brake=True, armed=True)

  def test_main_on_lands_on_one_sample(self):
    CI = car_interface()
    pk, n = self.armed_and_silent(CI)
    _, n = self.feed_pedals(CI, pk, n, MAIN_OFF_DEBOUNCE_SAMPLES, brake=True)
    ret, n = self.feed_pedals(CI, pk, n, 1, brake=True, armed=True)
    assert ret.cruiseState.available

  def test_debounce_counts_pedals_samples(self):
    # the panda counts PEDALS messages: a carstate frame without a fresh sample (a late or lost
    # PEDALS) must not count, or carstate's main falls ahead of the panda's
    CI = car_interface()
    pk, n = self.armed_and_silent(CI)
    ret = None
    for i in range(n, n + 2 * (MAIN_OFF_DEBOUNCE_SAMPLES - 1)):
      msgs = [pk.make_can_msg("ENGINE_DATA", 0, {"SPEED": 0})]
      if (i - n) % 2 == 0:
        msgs.append(pk.make_can_msg("PEDALS", 0, {"ACC_OFF": 0, "BRAKE_ON": 1}))
      ret, _ = feed(CI, i, *msgs)
    assert ret.cruiseState.available

  def test_debounce_is_the_panda_s(self):
    import os
    import re
    import opendbc.safety
    header = open(os.path.join(os.path.dirname(opendbc.safety.__file__), "modes", "mazda.h")).read()
    samples = int(re.search(r"#define MAZDA_MAIN_OFF_DEBOUNCE\s+(\d+)U", header).group(1))
    assert samples == MAIN_OFF_DEBOUNCE_SAMPLES


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


class TestRejectionReport:
  """The panda puts every frame its tx hook refused back on the can stream with src = bus + 0xC0;
  carstate counts our refused torque requests so the controller can restart its ramp."""

  @staticmethod
  def lkas(request):
    return packer().make_can_msg("CAM_LKAS", 0, {"LKAS_REQUEST": request})[1]

  def test_a_refused_torque_request_is_counted_once(self):
    rig = UndeliveredRig()
    rig.step(0, 0, 0)
    assert rig.CS.lkas_rejected == 0
    feed(rig.CI, rig.frame + 1, (0x243, self.lkas(312), 192))
    assert rig.CS.lkas_rejected == 1
    # the count is per cycle: nothing refused next frame means zero
    rig.step(0, 0, 0)
    assert rig.CS.lkas_rejected == 0

  def test_several_in_one_cycle_are_all_counted(self):
    rig = UndeliveredRig()
    feed(rig.CI, 1, (0x243, self.lkas(312), 192), (0x243, self.lkas(324), 192))
    assert rig.CS.lkas_rejected == 2

  def test_only_the_rejected_bus_counts(self):
    # the camera's own 0x243 (bus 2), the panda's echo of an accepted one (bus 128) and a frame
    # on the main bus are not rejections
    rig = UndeliveredRig()
    for src in (0, 2, 128):
      feed(rig.CI, 1, (0x243, self.lkas(312), src))
      assert rig.CS.lkas_rejected == 0

  def test_a_refused_zero_request_is_not_counted(self):
    # the panda refuses every LKA frame while it is not controlling; those carry nothing
    rig = UndeliveredRig()
    feed(rig.CI, 1, (0x243, self.lkas(0), 192), (0x243, self.lkas(0), 192))
    assert rig.CS.lkas_rejected == 0
    feed(rig.CI, 2, (0x243, self.lkas(0), 192), (0x243, self.lkas(-12), 192))
    assert rig.CS.lkas_rejected == 1

  def test_the_report_never_touches_can_validity(self):
    # nothing is ever refused on a clean drive, and the parser must not miss it
    rig = UndeliveredRig()
    for _ in range(300):  # 3 s, past every timeout the parser knows
      rig.step(300, 300, 0)
    loopback = rig.CI.can_parsers[Bus.loopback]
    assert loopback.can_valid
    assert not loopback.bus_timeout


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

  def test_lane_keep_setting_off_clears_and_inhibits_the_latch(self):
    # With the car's own lane-keep setting off the EPS applies nothing by design (route
    # 00000105): the latch clears, cannot re-arm, and the alert never fires. Both the CAM_SETTINGS
    # reading and LANE_LINES 0 count as the setting being off.
    for off in ({"LKAS_INERVENTION_ON1": 0, "ILKAS_NTERVENTION_ON2": 0}, None):
      rig = UndeliveredRig()
      for _ in range(rig.params.STEER_UNDELIVERED_FRAMES + 5):
        rig.step(600, 0, 1)
      assert rig.CS.steer_undelivered
      if off is not None:
        msg = rig.packer.make_can_msg("CAM_SETTINGS", 2, off)
      else:
        msg = rig.packer.make_can_msg("CAM_LANEINFO", 2, {"LANE_LINES": 0})
      rig.frame += 1
      ret, _ = feed(rig.CI, rig.frame, msg)
      assert ret.invalidLkasSetting
      hold = rig.params.STEER_UNDELIVERED_FRAMES + rig.params.STEER_UNDELIVERED_ALERT_FRAMES + 50
      for _ in range(hold):
        ret = rig.step(600, 0, 1)
      assert not rig.CS.steer_undelivered
      assert not ret.steerFaultTemporary
      if off is not None:
        # the setting back on: the latch measures again
        rig.frame += 1
        feed(rig.CI, rig.frame, rig.packer.make_can_msg("CAM_SETTINGS", 2, {"LKAS_INERVENTION_ON1": 1, "ILKAS_NTERVENTION_ON2": 1}))
        for _ in range(rig.params.STEER_UNDELIVERED_FRAMES + 5):
          rig.step(600, 0, 1)
        assert rig.CS.steer_undelivered

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
    self._btns(CI, pk, 0, TJA_BUTTON=0, BIT1=1)
    ret = self._btns(CI, pk, 1, TJA_BUTTON=1, BIT1=1)
    assert [be.type for be in ret.buttonEvents] == [self.ButtonType.lkas]
    assert ret.buttonEvents[0].pressed

  def test_no_event_without_the_button(self):
    CI, pk = self._declared(), packer()
    for i in range(10):
      ret = self._btns(CI, pk, i, TJA_BUTTON=0, BIT1=1)
      assert not [be for be in ret.buttonEvents if be.type == self.ButtonType.lkas]

  def test_undeclared_ignores_the_bit(self):
    CI, pk = car_interface(alpha_long=False), packer()
    self._btns(CI, pk, 0, TJA_BUTTON=0)
    ret = self._btns(CI, pk, 1, TJA_BUTTON=1)
    assert not [be for be in ret.buttonEvents if be.type == self.ButtonType.lkas]

  @pytest.mark.parametrize("mode_x, mode_y", [(1, 1), (0, 1), (1, 0)], ids=["both", "mode_y", "ke_mode_x"])
  def test_main_cruise_event(self, mode_x, mode_y):
    # either bit alone is a whole main press
    CI, pk = car_interface(alpha_long=False), packer()
    self._btns(CI, pk, 0, MODE_X=0, MODE_Y=0)
    ret = self._btns(CI, pk, 1, MODE_X=mode_x, MODE_Y=mode_y)
    assert [be.type for be in ret.buttonEvents] == [self.ButtonType.mainCruise]


class TestFirstEngageHold:
  """The EPS's first LKAS engagement after power-up raised LKAS_FAULT 250-300 ms after the first
  nonzero request on routes 0000001a, 000001bb and 000001e8: standby (block and track), zero
  delivery, 0.28-0.30 m/s. On the ten other first engagements from that state it delivered
  nothing until the standby lifted above 1 m/s, so a zero request there withholds no assist.
  carstate derives the hold; once the EPS has delivered once the same standby is left alone."""

  def crawl(self, rig, effective=0):
    return rig.step(100, effective, 1, speed_kph=1., track_state=1)

  def test_holds_through_the_first_standby_and_latches_on_delivery(self):
    rig = UndeliveredRig()
    self.crawl(rig)  # the parsers take a frame to fill
    for _ in range(30):
      self.crawl(rig)
      assert rig.CS.steer_first_engage_hold
    self.crawl(rig, effective=8)
    assert rig.CS.lkas_delivered
    for _ in range(30):
      self.crawl(rig)
      assert not rig.CS.steer_first_engage_hold  # route_118 t248, 12c t125: the same standby later in the drive

  @pytest.mark.parametrize("release", [dict(speed_kph=3.7), dict(blocked=0), dict(track_state=0)])
  def test_rolling_or_leaving_standby_releases(self, release):
    rig = UndeliveredRig()
    self.crawl(rig)
    self.crawl(rig)
    assert rig.CS.steer_first_engage_hold
    kw = dict(speed_kph=1., track_state=1, blocked=1)
    kw.update(release)
    rig.step(100, 0, kw.pop('blocked'), **kw)
    assert not rig.CS.steer_first_engage_hold


def test_intervention_bits_become_an_invalid_lkas_setting():
  CI, pk = car_interface(alpha_long=False), packer()
  healthy = pk.make_can_msg("CAM_SETTINGS", 2, {"LKAS_INERVENTION_ON1": 1, "ILKAS_NTERVENTION_ON2": 1})
  assert not feed(CI, 0, healthy)[0].invalidLkasSetting
  off = pk.make_can_msg("CAM_SETTINGS", 2, {"LKAS_INERVENTION_ON1": 0, "ILKAS_NTERVENTION_ON2": 0})
  assert feed(CI, 1, off)[0].invalidLkasSetting
  # the flag holds through the silent cycles between the message's arrivals
  for i in range(2, 12):
    assert feed(CI, i)[0].invalidLkasSetting
  # either bit alone is a live setting: cameras assert them unevenly, so the
  # single-bit states must not read as off
  on1_only = pk.make_can_msg("CAM_SETTINGS", 2, {"LKAS_INERVENTION_ON1": 1, "ILKAS_NTERVENTION_ON2": 0})
  assert not feed(CI, 12, on1_only)[0].invalidLkasSetting
  on2_only = pk.make_can_msg("CAM_SETTINGS", 2, {"LKAS_INERVENTION_ON1": 0, "ILKAS_NTERVENTION_ON2": 1})
  assert not feed(CI, 13, on2_only)[0].invalidLkasSetting


def test_faulted_camera_is_not_the_las_switch():
  # A faulted camera sends LANE_LINES 0 like the LAS switch, but with ERR_BIT set. Route
  # 55b9388c74f60c82/00000157 (AU CX-5 2022) held this frame from power-on: read as the switch, the
  # mici showed "LKAS is off" over the camera fault.
  CI = car_interface(alpha_long=False)
  las_off = bytes([0x42, 0, 0, 0, 0, 0, 0, 0])
  faulted = bytes.fromhex("4300000000011040")
  assert feed(CI, 0, (CAM_LANEINFO, las_off, 2))[0].invalidLkasSetting
  assert not feed(CI, 1, (CAM_LANEINFO, faulted, 2))[0].invalidLkasSetting

  # CAM_LKAS ERR_BIT_1 alone (steerFaultPermanent) also marks the camera as faulted; same route's frames
  CI = car_interface(alpha_long=False)
  feed(CI, 0, (CAM_LKAS, bytes.fromhex("f8000960020000a7"), 2))
  feed(CI, 1, (CAM_LKAS, bytes.fromhex("08000960020000b6"), 2))
  ret = feed(CI, 2, (CAM_LANEINFO, las_off, 2))[0]
  assert ret.steerFaultPermanent
  assert not ret.invalidLkasSetting


class TestLkasRearm:
  """lkas_arming spans the EPS's re-arm after the car's lane keep comes back on (LKAS_REARM_T)."""

  SETTING_OFF = {"LKAS_INERVENTION_ON1": 0, "ILKAS_NTERVENTION_ON2": 0}
  SETTING_ON = {"LKAS_INERVENTION_ON1": 1, "ILKAS_NTERVENTION_ON2": 1}

  def setting(self, rig, values):
    rig.frame += 1
    return feed(rig.CI, rig.frame, rig.packer.make_can_msg("CAM_SETTINGS", 2, values))[0]

  def off_then_on(self, rig):
    assert self.setting(rig, self.SETTING_OFF).invalidLkasSetting
    assert not rig.CS.lkas_arming
    for _ in range(50):
      rig.step(0, 0, 0)
    assert not self.setting(rig, self.SETTING_ON).invalidLkasSetting
    assert rig.CS.lkas_arming

  def test_armed_through_the_block_and_back_when_it_lifts(self):
    rig = UndeliveredRig()
    self.off_then_on(rig)
    for _ in range(LKAS_REARM_FRAMES + 7):
      ret = rig.step(600, 0, 1, track_state=1)
      assert rig.CS.lkas_arming
      assert not ret.steerFaultTemporary
    rig.step(0, 0, 0)
    assert not rig.CS.lkas_arming

  def test_the_gap_before_the_block_rises_does_not_end_it(self):
    # route 00000105: no block while the setting is off, set 0.02 s after the edge
    rig = UndeliveredRig()
    self.off_then_on(rig)
    rig.step(0, 0, 0)
    rig.step(0, 0, 0)
    assert rig.CS.lkas_arming
    for _ in range(LKAS_REARM_FRAMES - 3):
      rig.step(0, 0, 1, track_state=1)
    assert rig.CS.lkas_arming

  def test_a_clear_block_past_the_window_ends_it_without_a_request(self):
    # a straight road: requests under STEER_UNDELIVERED_MIN deliver nothing even when healthy
    rig = UndeliveredRig()
    self.off_then_on(rig)
    for _ in range(LKAS_REARM_FRAMES):
      rig.step(50, 0, 0)
    assert not rig.CS.lkas_arming

  def test_delivery_ends_it_early(self):
    rig = UndeliveredRig()
    self.off_then_on(rig)
    rig.step(600, 200, 0)
    assert not rig.CS.lkas_arming

  def test_switched_off_again_hands_back_to_the_setting(self):
    rig = UndeliveredRig()
    self.off_then_on(rig)
    assert self.setting(rig, self.SETTING_OFF).invalidLkasSetting
    assert not rig.CS.lkas_arming

  def test_the_las_switch_arms_it_too(self):
    # on a CX-5 2022 the LAS switch reads as LANE_LINES 0 (0x9e, routes 0000024d, 0000027c)
    rig = UndeliveredRig()
    rig.frame += 1
    assert feed(rig.CI, rig.frame, rig.packer.make_can_msg("CAM_LANEINFO", 2, {"LANE_LINES": 0}))[0].invalidLkasSetting
    rig.frame += 1
    assert not feed(rig.CI, rig.frame, rig.packer.make_can_msg("CAM_LANEINFO", 2, {"LANE_LINES": 2}))[0].invalidLkasSetting
    assert rig.CS.lkas_arming

  def test_never_armed_without_the_setting_going_off(self):
    rig = UndeliveredRig()
    assert not self.setting(rig, self.SETTING_ON).invalidLkasSetting
    for _ in range(20):
      rig.step(600, 0, 1, track_state=1)
      assert not rig.CS.lkas_arming

  def test_the_older_eps_does_not_soft_disable_over_the_rearm(self):
    # its steerFaultTemporary is LKAS_BLOCK at road speed: 3 s of it is a soft disable, and MADS
    # would lose the lateral it is handing back. Past the window a block is a fault again.
    rig = UndeliveredRig()
    rig.CI = car_interface(alpha_long=False, candidate=CAR.MAZDA_CX5)
    rig.CS = rig.CI.CS
    for _ in range(3):  # its LKAS speed gate reads ENGINE_DATA; one lone frame did not register
      rig.frame += 1
      feed(rig.CI, rig.frame, rig.packer.make_can_msg("ENGINE_DATA", 0, {"SPEED": 80.}))
    for _ in range(5):
      rig.step(0, 0, 0, speed_kph=80.)
    assert rig.step(0, 0, 1, speed_kph=80.).steerFaultTemporary, "a block at speed is a fault on this EPS"
    rig.step(0, 0, 0, speed_kph=80.)
    self.off_then_on(rig)
    for _ in range(LKAS_REARM_FAULT_FRAMES - 60):
      assert not rig.step(600, 0, 1, speed_kph=80.).steerFaultTemporary
    for _ in range(60):
      ret = rig.step(600, 0, 1, speed_kph=80.)
    assert ret.steerFaultTemporary


def test_cam_settings_absence_never_reads_as_off():
  # the parser's default zeros must not read as off on a car that never sends CAM_SETTINGS
  CI = car_interface(alpha_long=False)
  for i in range(10):
    assert not feed(CI, i)[0].invalidLkasSetting


class TestMrccButtonEvent:
  """The wheel's MRCC master press (CRZ_BTNS BIT1, active low) as a buttonEvent.

  Declared cars publish mainCruise like any master button; undeclared cars never read the
  bit, so an unknown idle level there cannot produce events.
  """

  def test_press_publishes_maincruise_on_a_declared_car(self):
    CI = car_interface()
    CI.CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON
    pk = packer()
    ret, _ = feed(CI, 0, pk.make_can_msg("CRZ_BTNS", 0, {"BIT1": 1}))
    assert not ret.buttonEvents
    ret, _ = feed(CI, 1, pk.make_can_msg("CRZ_BTNS", 0, {"BIT1": 0, "BIT1_INV": 1}))
    assert [(be.type, be.pressed) for be in ret.buttonEvents] == [(ButtonType.mainCruise, True)]

  def test_press_is_invisible_on_an_undeclared_car(self):
    CI = car_interface()
    pk = packer()
    ret, _ = feed(CI, 0, pk.make_can_msg("CRZ_BTNS", 0, {"BIT1": 1}))
    ret, _ = feed(CI, 1, pk.make_can_msg("CRZ_BTNS", 0, {"BIT1": 0, "BIT1_INV": 1}))
    assert not ret.buttonEvents
    assert CI.CS.mrcc_button == 0


class TestOceaniaCluster:
  """A NZ CX-9 2021 cluster shows the held cruise speed over-read: displayed = held / 0.98 + 1
  (zoompilot/opendbc#7, four samples). speed stays the CAN value, speedCluster is the dash."""

  SAMPLES = [(6176, 32), (7352, 38), (10098, 52), (19504, 100)]  # raw CRZ_SPEED, cluster km/h

  @staticmethod
  def _feed(CI, raw):
    # the parser wants two counted frames before it trusts the message
    pk = packer()
    ret = None
    for i in range(2):
      ret = feed(CI, i, pk.make_can_msg("CRZ_EVENTS", 0, {"CRZ_SPEED": raw / 200. - 0.5, "CTR": i}))[0]
    return ret

  @pytest.mark.parametrize("raw, cluster", SAMPLES)
  def test_oceania_publishes_the_dash_number_as_speed_cluster(self, raw, cluster):
    CI = car_interface(alpha_long=False, candidate=CAR.MAZDA_CX9_2021)
    CI.CP_SP.flags |= MazdaFlagsSP.OCEANIA_CLUSTER
    ret = self._feed(CI, raw)
    assert round(ret.cruiseState.speedCluster * CV.MS_TO_KPH) == cluster
    assert abs(ret.cruiseState.speed * CV.MS_TO_KPH - (raw / 200. - 0.5)) < 0.01

  @pytest.mark.parametrize("raw, cluster", SAMPLES)
  def test_other_clusters_show_the_can_value(self, raw, cluster):
    CI = car_interface(alpha_long=False, candidate=CAR.MAZDA_CX9_2021)
    ret = self._feed(CI, raw)
    assert ret.cruiseState.speedCluster == ret.cruiseState.speed

  def test_the_flag_follows_the_oceania_wmi(self):
    for wmi, expect in (("JM0", True), ("JM3", False), ("JM7", False), ("RN2", False)):
      CP = car_params(CAR.MAZDA_CX9_2021)
      CP.carVin = wmi + "TCBDY5M0123456"[:14]
      CP_SP = car_params_sp(CP, CAR.MAZDA_CX9_2021)
      assert bool(CP_SP.flags & MazdaFlagsSP.OCEANIA_CLUSTER) == expect, wmi


class TestDistanceButtons:
  """DISTANCE_LESS is upstream's one gapAdjustCruise button; DISTANCE_MORE is read as a level
  for the fork's opposite-direction step and never becomes a button event."""

  def _press(self, CI, less, more, i):
    pk = packer()
    return feed(CI, i, pk.make_can_msg("CRZ_BTNS", 0, {"DISTANCE_LESS": less, "DISTANCE_MORE": more, "CTR": i % 16}))[0]

  def test_closer_is_the_gap_button_and_farther_is_a_level(self):
    CI = car_interface(alpha_long=True)
    self._press(CI, 0, 0, 0)
    ret = self._press(CI, 1, 0, 1)
    assert [be.type for be in ret.buttonEvents] == [structs.CarState.ButtonEvent.Type.gapAdjustCruise]
    assert CI.CS.distance_more_button == 0
    ret = self._press(CI, 0, 1, 2)
    assert CI.CS.distance_more_button == 1
    assert not any(be.type == structs.CarState.ButtonEvent.Type.gapAdjustCruise and be.pressed for be in ret.buttonEvents)
