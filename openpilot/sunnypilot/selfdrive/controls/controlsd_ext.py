"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom

from opendbc.car import structs
from opendbc.sunnypilot.car.interfaces import get_steer_slew_schedule
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import icbm_active
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import LIVE_TORQUE_PARAMETERS_SP_SERVICE
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_smoothing import LaneChangeSmoothing
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import LatControlTorque as LatControlTorqueV2
from openpilot.sunnypilot.selfdrive.controls.lib.steer_limit import classify
from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import resolved_tune_versions

# how long lateral control has to be inactive, continuously, before the torque tune follows
# a model change: a one-frame drop (a steer fault flicker) would otherwise swap and resume
# steering on a controller no inactive frame had primed
TUNE_SWAP_INACTIVE_FRAMES = round(0.5 / DT_CTRL)


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()
    self.lane_change_smoothing = LaneChangeSmoothing()

    # steer-limit classifier (lib/steer_limit.py): None on brands without carcontroller rate
    # limits and on angle-steered cars, where controlsd's flag is left untouched
    self._steer_slew_schedule = None
    if CP.steerControlType != structs.CarParams.SteerControlType.angle:
      self._steer_slew_schedule = get_steer_slew_schedule(CP)
    self._lat_active_last = False
    # frames in a row with CC.latActive false, as state_control decided it (note_lat_active);
    # the torque tune swap waits for TUNE_SWAP_INACTIVE_FRAMES of them
    self._inactive_frames = 0
    self._applied_torque_prev: float | None = None

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP', 'carStateSP', LIVE_TORQUE_PARAMETERS_SP_SERVICE]
    self.pm_services_ext = ['carControlSP']

  @property
  def pcm_cruise_speed(self) -> bool:
    """CarParamsSP.pcmCruiseSpeed as card has it this frame: False while ICBM drives the set speed."""
    return not icbm_active(self.sm['carStateSP'], self.CP_SP)

  def initialize_lateral_control(self, lac, CI, dt):
    """One controller per model size, built once; every drive starts on the small model's."""
    # the enforce-off v0 forcing and the unset-param defaults both live in the resolver,
    # shared with the settings UIs so they gate on the tune that will actually run
    versions = resolved_tune_versions(self.params, self.CP.lateralTuning.which() == 'torque')

    def build(version):
      if version == 0.0:
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
      if version == 2.0:
        return LatControlTorqueV2(self.CP, self.CP_SP, CI, dt)
      return lac

    built = {version: build(version) for version in dict.fromkeys(versions.values())}
    self._lac_by_size = {big: built[version] for big, version in versions.items()}
    self._lacs = tuple(built.values())
    return self._lac_by_size[False]

  def select_lateral_control(self, sm: messaging.SubMaster) -> None:
    """Runs at the end of every frame. modelV2.big says which model produced the frame; a
    change swaps self.LaC to the controller tuned for it, reset, but never while lateral
    control is active this frame. The idle controller holds whatever it had when it last
    steered (its request buffer, previous measurement, integrator): on the 2026-09-29 drives
    every big-to-small hand-back while steering stepped the commanded torque by 0.09 to 0.19
    of full scale in one tick. Since a hand-back no longer ends in a soft disable, the small
    model is carried by the tune that was steering until lateral has been inactive for
    TUNE_SWAP_INACTIVE_FRAMES in a row (a disengage, a blinker pause, a stop); a shorter drop
    keeps it. A swap to the big model only happens with nothing in control, and its second of
    no-entry is longer than the wait. From the next frame state_control runs the incoming
    controller, inactive until an engagement, which primes it as for any engagement, and
    pushes the live torque params, modelV2 and the lag into it. An engagement on the very
    frame after the swap would find it unprimed; priming it here would need the frame's
    CarState, which state_control has and this does not."""
    if len(self._lacs) == 1 or self._inactive_frames < TUNE_SWAP_INACTIVE_FRAMES:
      return
    big = bool(sm['modelV2'].big)
    lac = self._lac_by_size[big]
    if lac is self.LaC:
      return
    self.LaC = lac
    lac.reset()
    cloudlog.warning("controlsd: %s model, swapped to its torque tune", "big" if big else "small")

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()
      self.lane_change_smoothing.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    self._lat_active_last = self._get_lat_active(sm)
    return self._lat_active_last

  def _get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  def reclassify_steer_limit(self, sm: messaging.SubMaster) -> None:
    """Runs after publish() has set this frame's steer_limited_by_safety from the raw torque
    mismatch. Replaces it with the classifier's directional, rail-aware flag (lib/steer_limit.py)
    before the next frame's LaC.update reads it. Torque tunes only; while lateral is inactive
    (get_lat_active's last value, the one publish() gated on) the flag is left as controlsd set it."""
    ext = getattr(self.LaC, 'extension', None)
    if ext is None or self._steer_slew_schedule is None:
      return
    if not self._lat_active_last:
      self._applied_torque_prev = None
      return
    v_ego = sm['carState'].vEgo
    applied = float(sm['carOutput'].actuatorsOutput.torque)
    bp, up, down = self._steer_slew_schedule
    rail_scale = ext.rail_scale_at(v_ego)
    limit = classify(ext.commanded_torque, applied, self._applied_torque_prev,
                     float(np.interp(v_ego, bp, up)), float(np.interp(v_ego, bp, down)),
                     rail_scale, self.steer_limited_by_safety, ext.last_error, ext.integrator)
    self.steer_limited_by_safety = limit.limited
    ext.set_actuator_state(applied, limit.at_rail)
    self._applied_torque_prev = applied

  def lane_change_jerk_factor(self, sm: messaging.SubMaster, lat_active: bool,
                              new_desired_curvature: float, prev_desired_curvature: float) -> float:
    """Lane-change smoothing's jerk factor for clip_curvature (1.0 outside a smoothed lane
    change). The lateral maneuver mode's scripted commands pass through the stock clip.
    Called once a frame with CC.latActive, which select_lateral_control waits on."""
    self.note_lat_active(lat_active)
    if sm.valid['lateralManeuverPlan']:
      # a lane-change unwind armed before maneuver mode must not resume stale after it
      self.lane_change_smoothing.reset()
      return 1.0
    return self.lane_change_smoothing.update(sm['carState'], sm['modelV2'], lat_active, new_desired_curvature, prev_desired_curvature)

  def note_lat_active(self, lat_active: bool) -> None:
    self._inactive_frames = 0 if lat_active else self._inactive_frames + 1

  @staticmethod
  def get_lead_data(_lead, src: log.RadarState.LeadData) -> None:
    _lead.dRel = src.dRel
    _lead.yRel = src.yRel
    _lead.vRel = src.vRel
    _lead.aRel = src.deprecated.aRel
    _lead.vLead = src.vLead
    _lead.dPath = src.deprecated.dPath
    _lead.vLat = src.deprecated.vLat
    _lead.vLeadK = src.vLeadK
    _lead.aLeadK = src.aLeadK
    _lead.fcw = src.deprecated.fcw
    _lead.status = src.present
    _lead.aLeadTau = src.aLeadTau
    _lead.modelProb = src.modelProb
    _lead.radar = src.radar
    _lead.radarTrackId = src.radarTrackId

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    self.get_lead_data(CC_SP.leadOne, sm['radarState'].leadOne)
    self.get_lead_data(CC_SP.leadTwo, sm['radarState'].leadTwo)

    # MADS state
    mads_src = sm['selfdriveStateSP'].mads
    CC_SP.mads.state = mads_src.state
    CC_SP.mads.enabled = mads_src.enabled
    CC_SP.mads.active = mads_src.active
    CC_SP.mads.available = mads_src.available

    # ICBM state
    icbm_src = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.intelligentCruiseButtonManagement.state = icbm_src.state
    CC_SP.intelligentCruiseButtonManagement.sendButton = icbm_src.sendButton
    CC_SP.intelligentCruiseButtonManagement.vTarget = icbm_src.vTarget

    # lane-change pace clamp telemetry, for offline validation
    CC_SP.zoompilot.laneChangeSmoothing.jerkFactor = float(self.lane_change_smoothing.jerk_factor)

    return CC_SP

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
    self.reclassify_steer_limit(sm)

    # Speed-dependent torque: apply per-bin learned values to the lateral controller
    if (self.CP.lateralTuning.which() == 'torque'
        and sm.updated.get('lateralTorqueParameters', False)
        and sm.all_checks(['lateralTorqueParameters'])):
      tp = sm['lateralTorqueParameters']
      # torqued_ext publishes the bins beside every upstream message on the fork service;
      # one that has not checked out counts as no bins
      tp_sp = sm[LIVE_TORQUE_PARAMETERS_SP_SERVICE] if sm.all_checks([LIVE_TORQUE_PARAMETERS_SP_SERVICE]) else None
      # both sizes' controllers, so the idle one holds current bins when it takes over.
      # handles activation AND deactivation: useParams off or empty bins de-assert
      for lac in self._lacs:
        lac.extension.update_speed_dep_torque(tp, tp_sp)

    self.select_lateral_control(sm)
