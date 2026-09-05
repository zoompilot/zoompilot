"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import functools
import json
import os
import numpy as np
from typing import NamedTuple
from collections.abc import Callable

from opendbc.car import structs
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.subaru.values import SubaruFlags
from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.sunnypilot.car.hyundai.enable_radar_tracks import enable_radar_tracks as hyundai_enable_radar_tracks
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import LongitudinalTuningType
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP, MazdaSafetyFlagsSP
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP, SubaruSafetyFlagsSP
from opendbc.sunnypilot.car.tesla.values import MadsScreenButtonType, TeslaFlagsSP, TeslaSafetyFlagsSP
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP


class LatControlInputs(NamedTuple):
  lateral_acceleration: float
  roll_compensation: float
  vego: float
  aego: float


TorqueFromLateralAccelCallbackTypeTorqueSpace = Callable[[LatControlInputs, structs.CarParams.LateralTorqueTuning, bool], float]


@functools.cache
def get_speed_dep_config():
  """Load speed-dependent torque config from toml. Cached after first call."""
  import tomllib
  from pathlib import Path
  from opendbc.car.common.basedir import BASEDIR
  path = Path(BASEDIR) / 'torque_data/speed_dependent.toml'
  with open(path, 'rb') as f:
    return tomllib.load(f)


def get_steer_max_schedule(CP):
  """The carcontroller's normalized-torque-to-CAN-counts scale by speed, read from the
  brand's CarControllerParams. Returns (speed_bp, steer_max_v), or None when the brand
  has no speed-dependent STEER_MAX (a flat scale cancels out of per-count LAF math, so
  consumers skip the normalization entirely)."""
  try:
    values = __import__(f'opendbc.car.{CP.brand}.values', fromlist=['CarControllerParams'])
    ccp = values.CarControllerParams(CP)
  except (ImportError, AttributeError, TypeError):
    return None
  lookup = getattr(ccp, 'STEER_MAX_LOOKUP', None)
  if lookup is None:
    return None
  return [float(x) for x in lookup[0]], [float(x) for x in lookup[1]]


def get_steer_rail_schedule(CP):
  """Normalized fraction of the carcontroller's steer scale the EPS will actually deliver,
  by speed: EPS_CEILING_LOOKUP / STEER_MAX(v), piecewise-linear on the union of both
  schedules' breakpoints, clipped to 1.0. None when the brand declares no ceiling (the
  EPS delivers the full scale everywhere). Lets a lateral controller treat reaching the
  measured rail as actuator saturation instead of comparing against a full-scale command
  it can never deliver above the ceiling's falloff."""
  try:
    values = __import__(f'opendbc.car.{CP.brand}.values', fromlist=['CarControllerParams'])
    ccp = values.CarControllerParams(CP)
  except (ImportError, AttributeError, TypeError):
    return None
  ceiling = getattr(ccp, 'EPS_CEILING_LOOKUP', None)
  if ceiling is None:
    return None
  sm_lookup = getattr(ccp, 'STEER_MAX_LOOKUP', None)
  if sm_lookup is not None:
    sm_bp, sm_v = [float(x) for x in sm_lookup[0]], [float(x) for x in sm_lookup[1]]
  else:
    sm_bp, sm_v = [0.0], [float(ccp.STEER_MAX)]
  ceil_bp, ceil_v = [float(x) for x in ceiling[0]], [float(x) for x in ceiling[1]]
  bp = sorted(set(ceil_bp + sm_bp))
  rail = [min(1.0, float(np.interp(v, ceil_bp, ceil_v)) / float(np.interp(v, sm_bp, sm_v))) for v in bp]
  return bp, rail


def get_steer_slew_schedule(CP):
  """Per-frame normalized torque slew the carcontroller allows, by speed:
  (speed_bp, up, down) with up = STEER_DELTA_UP / STEER_MAX(v) and down = STEER_DELTA_DOWN /
  STEER_MAX(v), on STEER_MAX_LOOKUP's breakpoints when the scale is speed-dependent and on a
  single breakpoint otherwise. Lets controlsd's steer-limit classifier tell a command the
  actuator is still walking toward (one slew step behind) from one the driver envelope or
  the EPS rail is holding back. None when the brand's CarControllerParams lacks the
  attributes or cannot be built from CP (the consumer keeps upstream's flag as is)."""
  try:
    values = __import__(f'opendbc.car.{CP.brand}.values', fromlist=['CarControllerParams'])
    ccp = values.CarControllerParams(CP)
  except (ImportError, AttributeError, TypeError):
    return None
  delta_up = getattr(ccp, 'STEER_DELTA_UP', None)
  delta_down = getattr(ccp, 'STEER_DELTA_DOWN', None)
  if delta_up is None or delta_down is None:
    return None
  lookup = getattr(ccp, 'STEER_MAX_LOOKUP', None)
  if lookup is not None:
    bp, sm_v = [float(x) for x in lookup[0]], [float(x) for x in lookup[1]]
  else:
    steer_max = getattr(ccp, 'STEER_MAX', None)
    if steer_max is None:
      return None
    bp, sm_v = [0.0], [float(steer_max)]
  return bp, [float(delta_up) / sm for sm in sm_v], [float(delta_down) / sm for sm in sm_v]


def get_speed_dep_config_for_car(CP):
  """The speed-dep entry for this car, honoring the entry's validity predicate.

  An entry measured on a zero-min-steer-speed EPS (e.g. an EPS-swapped car) declares
  requires_steer_to_zero: its LAF values were learned under that EPS's STEER_MAX
  schedule, and the same model with its stock EPS runs a different schedule, so the
  seeds would be mis-scaled there. minSteerSpeed == 0 is the brand-neutral statement
  that the EPS steers to a stop, which is what the entry requires.

  An active entry carries the platform's STEER_MAX schedule under 'steer_max_schedule'
  when one exists: bin LAF values are normalized units learned under one scale each,
  so a consumer interpolating across bins needs the schedule to do it in per-count
  space instead of smearing the scale's step across the bin span."""
  cfg = get_speed_dep_config().get(CP.carFingerprint, {})
  if cfg.get('requires_steer_to_zero') and CP.minSteerSpeed > 0:
    return {}
  cfg = dict(cfg)
  if cfg:
    schedule = get_steer_max_schedule(CP)
    if schedule is not None:
      cfg['steer_max_schedule'] = schedule
  return cfg


class CarInterfaceBaseSP:
  @staticmethod
  def torque_from_lateral_accel_linear_in_torque_space(latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                                        gravity_adjusted: bool) -> float:
    # The default is a linear relationship between torque and lateral acceleration (accounting for road roll and steering friction)
    return latcontrol_inputs.lateral_acceleration / float(torque_params.latAccelFactor)

  def torque_from_lateral_accel_in_torque_space(self) -> TorqueFromLateralAccelCallbackTypeTorqueSpace:
    return self.torque_from_lateral_accel_linear_in_torque_space

class NanoFFModel:
  def __init__(self, weights_loc: str, platform: str):
    self.weights_loc = weights_loc
    self.platform = platform
    self.load_weights(platform)

  def load_weights(self, platform: str):
    with open(self.weights_loc) as fob:
      self.weights = {k: np.array(v) for k, v in json.load(fob)[platform].items()}

  def relu(self, x: np.ndarray):
    return np.maximum(0.0, x)

  def forward(self, x: np.ndarray):
    assert x.ndim == 1
    x = (x - self.weights['input_norm_mat'][:, 0]) / (self.weights['input_norm_mat'][:, 1] - self.weights['input_norm_mat'][:, 0])
    x = self.relu(np.dot(x, self.weights['w_1']) + self.weights['b_1'])
    x = self.relu(np.dot(x, self.weights['w_2']) + self.weights['b_2'])
    x = self.relu(np.dot(x, self.weights['w_3']) + self.weights['b_3'])
    x = np.dot(x, self.weights['w_4']) + self.weights['b_4']
    return x

  def predict(self, x: list[float], do_sample: bool = False):
    x = self.forward(np.array(x))
    if do_sample:
      pred = np.random.laplace(x[0], np.exp(x[1]) / self.weights['temperature'])
    else:
      pred = x[0]
    pred = pred * (self.weights['output_norm_mat'][1] - self.weights['output_norm_mat'][0]) + self.weights['output_norm_mat'][0]
    return pred


def setup_interfaces(CI, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                     params_list: list[dict[str, str]] | None = None,
                     can_recv: CanRecvCallable | None = None, can_send: CanSendCallable | None = None) -> None:
  if params_list is None:
    params_list = []

  params_dict = {k: v for param in params_list for k, v in param.items()}

  _initialize_custom_longitudinal_tuning(CI, CP, CP_SP, params_dict)
  _initialize_coop_steering(CP, CP_SP, params_dict)
  _initialize_tesla_mads_screen_button(CP, CP_SP, params_dict)
  _initialize_radar_tracks(CP, CP_SP, can_recv, can_send)
  _initialize_stop_and_go(CP, CP_SP, params_dict)
  _initialize_toyota(CP, CP_SP, params_dict)
  _initialize_mazda(CP, CP_SP, params_dict)


def _initialize_custom_longitudinal_tuning(CI, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                           params_dict: dict[str, str]) -> None:

  # Hyundai Custom Longitudinal Tuning
  if CP.brand == 'hyundai':
    hyundai_longitudinal_tuning = int(params_dict.get("HyundaiLongitudinalTuning", 0))
    if hyundai_longitudinal_tuning == LongitudinalTuningType.DYNAMIC:
      CP_SP.flags |= HyundaiFlagsSP.LONG_TUNING_DYNAMIC.value
    if hyundai_longitudinal_tuning == LongitudinalTuningType.PREDICTIVE:
      CP_SP.flags |= HyundaiFlagsSP.LONG_TUNING_PREDICTIVE.value

  _ = CI.get_longitudinal_tuning_sp(CP, CP_SP)


def _initialize_coop_steering(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                              params_dict: dict[str, str]) -> None:
  if CP.brand == 'tesla':
    coop_steering = int(params_dict.get("TeslaCoopSteering", 0)) == 1
    if coop_steering:
      CP_SP.flags |= TeslaFlagsSP.COOP_STEERING.value


def _initialize_tesla_mads_screen_button(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                         params_dict: dict[str, str]) -> None:
  if CP.brand == 'tesla' and CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
    selection = int(params_dict.get("TeslaMadsScreenButton", MadsScreenButtonType.OFF))
    if selection == MadsScreenButtonType.THREE_FINGER:
      CP_SP.flags |= TeslaFlagsSP.MADS_SCREEN_BUTTON_3_FINGER.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_3_FINGER
    elif selection == MadsScreenButtonType.FOUR_FINGER:
      CP_SP.flags |= TeslaFlagsSP.MADS_SCREEN_BUTTON_4_FINGER.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_4_FINGER
    elif selection == MadsScreenButtonType.FIVE_FINGER:
      CP_SP.flags |= TeslaFlagsSP.MADS_SCREEN_BUTTON_5_FINGER.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_5_FINGER


def _initialize_radar_tracks(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                             can_recv: CanRecvCallable | None = None, can_send: CanSendCallable | None = None) -> None:
  if can_recv is None or can_send is None or os.environ.get("REPLAY"):
    return

  if CP.brand == 'hyundai':
    if CP.flags & HyundaiFlags.MANDO_RADAR and (CP.radarUnavailable or CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC):
      tracks_enabled = hyundai_enable_radar_tracks(can_recv, can_send, bus=0, addr=0x7d0)
      CP.radarUnavailable = not tracks_enabled


def _initialize_stop_and_go(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  if CP.brand == 'subaru' and not CP.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID):
    stop_and_go = int(params_dict.get("SubaruStopAndGo", 0)) == 1
    stop_and_go_manual_parking_brake = int(params_dict.get("SubaruStopAndGoManualParkingBrake", 0)) == 1

    if stop_and_go:
      CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO.value
    if stop_and_go_manual_parking_brake:
      CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE.value
    if stop_and_go or stop_and_go_manual_parking_brake:
      CP_SP.safetyParam |= SubaruSafetyFlagsSP.STOP_AND_GO


def _initialize_toyota(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  if CP.brand == 'toyota':
    toyota_stock_long = int(params_dict.get("ToyotaEnforceStockLongitudinal", 0)) == 1
    toyota_stop_and_go_hack = int(params_dict.get("ToyotaStopAndGoHack", 0)) == 1

    if toyota_stock_long:
      CP_SP.flags |= ToyotaFlagsSP.STOCK_LONGITUDINAL.value
      CP.alphaLongitudinalAvailable = False
      CP.openpilotLongitudinalControl = False
      CP.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    if toyota_stop_and_go_hack and CP.openpilotLongitudinalControl:
      CP_SP.flags |= ToyotaFlagsSP.STOP_AND_GO_HACK.value


def _initialize_mazda(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  if CP.brand == 'mazda':
    # The TJA button is fitted to some trims only and the fingerprint cannot tell, so the
    # driver declares it. With it, the button owns lateral and MRCC only controls cruise.
    if int(params_dict.get("MazdaTjaButton", 0)) == 1:
      CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON.value
      CP_SP.safetyParam |= MazdaSafetyFlagsSP.TJA_BUTTON
