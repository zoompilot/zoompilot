"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared harness for the SLA/arbiter rlog replays.

Builds a CruiseArbiter as card would and streams recorded frames through it:
carControl carries engagement, longitudinalPlanSP carries the resolver limits, and
every carState frame steps the arbiter with the recorded buttons and cluster.
"""
import glob

from openpilot.tools.lib.logreader import LogReader
from openpilot.common.params import Params
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.car.cruise_arbiter import CruiseArbiter
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

ButtonType = car.CarState.ButtonEvent.Type

BUTTON_MAP = {
  'accelCruise': ButtonType.accelCruise,
  'decelCruise': ButtonType.decelCruise,
  'setCruise': ButtonType.setCruise,
  'resumeCruise': ButtonType.resumeCruise,
}


def sorted_segments(route_glob: str) -> list[str]:
  return sorted(glob.glob(route_glob), key=lambda p: int(p.split('--')[-1].split('/')[0]))


def make_arbiter() -> CruiseArbiter:
  params = Params()
  params.put("IsReleaseSpBranch", True, block=True)
  params.put("SpeedLimitMode", int(Mode.assist), block=True)
  params.put_bool("IsMetric", False, block=True)

  CP = car.CarParams(pcmCruise=True, brand="mazda")
  CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
  arb = CruiseArbiter(CP, CP_SP)
  arb.read_params(params)
  assert arb.applicable
  return arb


def arbiter_frames(arb: CruiseArbiter, paths: list[str]):
  """Yield (t_rel, prev_state, recorded_carState) after stepping the arbiter on each
  recorded carState frame; t_rel is seconds from the first message of the route."""
  enabled = False
  lp = custom.LongitudinalPlanSP()
  t0 = None

  for path in paths:
    for msg in LogReader(path):
      which = msg.which()
      t_abs = msg.logMonoTime * 1e-9
      if t0 is None:
        t0 = t_abs

      if which == 'carControl':
        enabled = msg.carControl.enabled
      elif which == 'longitudinalPlanSP':
        r = msg.longitudinalPlanSP.speedLimit.resolver
        lp = custom.LongitudinalPlanSP()
        lp.speedLimit.resolver.speedLimit = r.speedLimit
        lp.speedLimit.resolver.speedLimitFinalLast = r.speedLimitFinalLast
        lp.speedLimit.resolver.speedLimitLastValid = r.speedLimitValid or r.speedLimitLastValid
      elif which == 'carState':
        cs = msg.carState
        CS = car.CarState()
        CS.buttonEvents = [car.CarState.ButtonEvent(type=BUTTON_MAP[str(b.type)], pressed=b.pressed)
                           for b in cs.buttonEvents if str(b.type) in BUTTON_MAP]

        arb.update_limit(lp)
        prev_state = arb.state
        arb.step(CS, enabled, cs.vCruise, cs.vCruiseCluster)
        yield t_abs - t0, prev_state, cs
