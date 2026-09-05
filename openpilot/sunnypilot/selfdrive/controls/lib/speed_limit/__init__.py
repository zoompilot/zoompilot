"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import custom

LIMIT_ADAPT_ACC = -1.  # m/s^2 Ideal acceleration for the adapting (braking) phase when approaching speed limits.
LIMIT_MAX_MAP_DATA_AGE = 10.  # s Maximum time to hold to map data, then consider it invalid inside limits controllers.

# Speed Limit Assist constants
PCM_LONG_REQUIRED_MAX_SET_SPEED = {
  True: (33.3333, 36.1111),  # km/h, (120, 130)
  False: (31.2928, 35.7632),  # mph, (70, 80)
}

CONFIRM_SPEED_THRESHOLD = {
  True: 80,   # km/h
  False: 50,  # mph
}

# The card-side cruise stack cannot import selfdrive.car.cruise (it imports cruise_ext at
# module top, before its constants exist), so this is the one fork copy of the unset
# sentinel; test_cruise_constants pins it to upstream's value.
V_CRUISE_UNSET = 255.

# shared by the pcm machine (plannerd), the cruise arbiter (card), and the mirror
_AssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
ACTIVE_STATES = (_AssistState.active, _AssistState.adapting)
ENABLED_STATES = (_AssistState.preActive, _AssistState.pending, *ACTIVE_STATES)
