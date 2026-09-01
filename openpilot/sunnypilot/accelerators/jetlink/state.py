"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Publishes the Jetson's health as `chestnutState`.

Reusing the message rather than adding one keeps the cereal schema, the
sidebar, the offroad alerts and the logging untouched: on the wire "chestnut"
means the active accelerator. The field mapping is documented in
jetlink/server/telemetry.py.

The values arrive piggybacked on the previous inference response, so publishing
costs no extra round trip and cannot delay a frame.
"""
from __future__ import annotations

import openpilot.cereal.messaging as messaging

# jetlink reports neutral field names; this is the only file that knows
# openpilot's schema, so the whole mapping lives here.
#
# chestnut reports LTSSM L0 (link trained) as 0x78 and openpilot alerts on
# anything else, so reuse the value: "link up" then means the same thing to
# every existing consumer.
LTSSM_L0 = 0x78

FLOAT_FIELDS = {
  'tempC': 'temp_c',
  'memoryTempC': 'memory_temp_c',
  'powerDrawW': 'power_w',
  'powerLimitW': 'power_limit_w',
}
# capnp integer bounds, so a bad sysfs read cannot raise mid-drive
INT_FIELDS = {
  'gpuUsagePercent': ('gpu_load_pct', 0, 255),
  'gpuClockMhz': ('gpu_clock_mhz', 0, 65535),
  'fanSpeedRpm': ('fan_rpm', 0, 65535),
  'supplyVoltage': ('supply_mv', 0, 65535),
  'supplyCurrent': ('supply_ma', -32768, 32767),
}


class JetlinkHealth:
  def __init__(self, pm: messaging.PubMaster, client):
    self.pm = pm
    self.client = client
    # False also when there is no client at all: the large model failed to load
    # and modeld is on the small one, so there is no link to report on. modeld
    # clears this too if the large model dies mid-drive.
    self.big = client is not None

  def send(self) -> None:
    msg = messaging.new_message('chestnutState')
    state = msg.chestnutState
    telemetry = self.client.last_state if self.big else None

    if telemetry:
      for field, key in FLOAT_FIELDS.items():
        if key in telemetry:
          setattr(state, field, float(telemetry[key]))
      for field, (key, lo, hi) in INT_FIELDS.items():
        if key in telemetry:
          setattr(state, field, max(lo, min(hi, int(telemetry[key]))))
      state.pcieLtssm = LTSSM_L0

    msg.valid = bool(telemetry) and not self.client.dead
    self.pm.send('chestnutState', msg)
