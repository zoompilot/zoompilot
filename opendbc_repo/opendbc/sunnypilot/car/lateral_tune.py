"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-car lateral tune data: the speed-dependent torque table and the carcontroller's steer
scale, rail and slew, as the controller and the torque learner read them.
"""
import functools


@functools.cache
def get_speed_dep_config():
  """Load speed-dependent torque config from toml. Cached after first call."""
  import tomllib
  from pathlib import Path
  from opendbc.car.common.basedir import BASEDIR
  path = Path(BASEDIR) / 'torque_data/speed_dependent.toml'
  with open(path, 'rb') as f:
    cfg = tomllib.load(f)
  # An entry may borrow another platform's table with 'substitute'; its own keys override.
  for name, entry in cfg.items():
    if 'substitute' in entry:
      cfg[name] = {**cfg[entry['substitute']], **{k: v for k, v in entry.items() if k != 'substitute'}}
  return cfg


def _controller_params_class(CP):
  """The brand's CarControllerParams class, or None when the brand has none."""
  try:
    return __import__(f'opendbc.car.{CP.brand}.values', fromlist=['CarControllerParams']).CarControllerParams
  except (ImportError, AttributeError):
    return None


def _controller_params(CP):
  """The brand's CarControllerParams built from CP, or None when it cannot be."""
  ccp = _controller_params_class(CP)
  try:
    return None if ccp is None else ccp(CP)
  except (ImportError, AttributeError, TypeError):
    return None


def get_tune_scale(CP) -> float:
  """How far the carcontroller's STEER_MAX sits from the one upstream-fitted torque values
  (params.toml, NNLC models, the manual override) are expressed on: a latAccelFactor is
  multiplied by it and a friction or an NNLC torque divided by it to put the same counts on the
  wire. The brand's CarControllerParams.TUNE_SCALE, 1.0 when it declares none."""
  return float(getattr(_controller_params_class(CP), 'TUNE_SCALE', 1.0))


def get_steer_rail_schedule(CP):
  """Normalized fraction of the carcontroller's steer scale the EPS will actually deliver,
  by speed: EPS_CEILING_LOOKUP / STEER_MAX, clipped to 1.0. None when the brand declares no
  ceiling (the EPS delivers the full scale everywhere). Lets a lateral controller treat
  reaching the measured rail as actuator saturation instead of comparing against a
  full-scale command it can never deliver above the ceiling's falloff."""
  ccp = _controller_params(CP)
  ceiling = getattr(ccp, 'EPS_CEILING_LOOKUP', None)
  if ceiling is None:
    return None
  return [float(x) for x in ceiling[0]], [min(1.0, float(x) / float(ccp.STEER_MAX)) for x in ceiling[1]]


def get_steer_slew_schedule(CP):
  """Per-frame normalized torque slew the carcontroller allows, by speed:
  (speed_bp, up, down) with up = STEER_DELTA_UP / STEER_MAX(v) and down = STEER_DELTA_DOWN /
  STEER_MAX(v), on STEER_MAX_LOOKUP's breakpoints when the scale is speed-dependent and on a
  single breakpoint otherwise. Lets controlsd's steer-limit classifier tell a command the
  actuator is still walking toward (one slew step behind) from one the driver envelope or
  the EPS rail is holding back. None when the brand's CarControllerParams lacks the
  attributes or cannot be built from CP (the consumer keeps upstream's flag as is)."""
  ccp = _controller_params(CP)
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

  An entry measured behind a zero-min-steer-speed EPS (e.g. an EPS-swapped car) declares
  requires_steer_to_zero: its values were learned behind that EPS firmware, and the same
  model on its stock EPS steers only above a floor, through the firmware's own dead band.
  minSteerSpeed == 0 is the brand-neutral statement that the EPS steers to a stop, which is
  what the entry requires. An entry that stays active for a car with a floor loses the bins
  centered below it, and 'min_speed' keeps the first remaining bin's lower edge where the
  full table puts it rather than widening that bin down to the default floor."""
  cfg = get_speed_dep_config().get(CP.carFingerprint, {})
  if cfg.get('requires_steer_to_zero') and CP.minSteerSpeed > 0:
    return {}
  cfg = dict(cfg)
  if cfg and CP.minSteerSpeed > 0 and 'speed_bp' in cfg:
    # A car never steers below its floor, so bins centered there hold seeds it can neither use
    # nor learn against.
    centers = cfg['speed_bp']
    keep = [i for i, v in enumerate(centers) if v >= CP.minSteerSpeed]
    if keep and keep[0] > 0:
      cfg['min_speed'] = (centers[keep[0] - 1] + centers[keep[0]]) / 2
    for key in ('speed_bp', 'laf_bp', 'friction_bp'):
      if key in cfg:
        if keep:
          cfg[key] = [cfg[key][i] for i in keep]
        else:
          del cfg[key]
  return cfg
