"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

comma's chestnut board: an AMD GPU over PCIe, behind an ASM USB bridge.

ChestnutState is upstream's, moved here unchanged so modeld holds no
backend-specific code. It stays in this process because only modeld may touch
the device.

tinygrad is imported where it is used rather than at module scope: hardwared,
the UI and the model manager all ask this backend whether a board is fitted,
and none of them should pay for tinygrad to find out.
"""
from __future__ import annotations

import ctypes
import os
import struct
from functools import cached_property

import openpilot.cereal.messaging as messaging
from openpilot.cereal.messaging import PubMaster
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.helpers import chestnut_compiled, chestnut_present

from openpilot.sunnypilot.accelerators.base import Daemon

# tinygrad's HCQ wait timeout. The default is tighter than a cold board's first
# queue submission, so modeld would fail to load a model it can actually run.
HCQ_WAIT_TIMEOUT_MS = '3000'


class ChestnutState:
  # only modeld can access chestnut
  def __init__(self, pm: PubMaster, big: bool):
    self.pm = pm
    self.big = big
    self.valid = True
    self.sends = 0
    self.metrics = {}

  @cached_property
  def power_limit(self) -> int:
    from tinygrad.device import Device
    smu = Device["AMD"].iface.dev_impl.smu
    return smu._send_msg(smu.smu_mod.PPSMC_MSG_GetPptLimit, 0, read_back_arg=True, timeout=100)

  def send(self) -> None:
    from tinygrad.device import Device
    msg = messaging.new_message('chestnutState')
    state = msg.chestnutState
    self.sends += 1
    if self.big and "AMD" in Device._opened_devices and self.sends % 100 == 1:
      try:
        smu = Device["AMD"].iface.dev_impl.smu
        metrics_t = smu.smu_mod.SmuMetricsExternal_t
        smu._send_msg(smu.smu_mod.PPSMC_MSG_TransferTableSmu2Dram, smu.smu_mod.TABLE_SMU_METRICS, timeout=100)
        metrics_buf = bytearray(smu.adev.vram.view(smu.driver_table_paddr, ctypes.sizeof(metrics_t))[:])
        metrics = metrics_t.from_buffer(metrics_buf).SmuMetrics
        self.metrics = {'tempC': metrics.AvgTemperature[smu.smu_mod.TEMP_HOTSPOT],
                        'memoryTempC': metrics.AvgTemperature[smu.smu_mod.TEMP_MEM],
                        'powerDrawW': metrics.AverageSocketPower,
                        'powerLimitW': self.power_limit,
                        'gpuUsagePercent': metrics.AverageGfxActivity,
                        'gpuClockMhz': metrics.AverageGfxclkFrequencyPostDs,
                        'fanSpeedRpm': metrics.AvgFanRpm}
        self.valid = True
      except Exception:
        if self.valid:
          cloudlog.exception("chestnut state read failed")
        self.valid = False
        self.metrics.clear()
    if self.big:
      for k, v in self.metrics.items():
        setattr(state, k, v)

    asm_valid = False
    if "AMD" in Device._opened_devices:
      try:
        # ASM runs on USB-C power, these still read without a gpu
        asm = Device["AMD"].iface.pci_dev.usb
        state.pcieLtssm = asm.read(0xB450, 1)[0]
        state.supplyVoltage, state.supplyCurrent = struct.unpack('<Hh', bytes(asm.usb.control_read(0xC0, 5))[:4])
        asm_valid = True
      except Exception:
        pass

    msg.valid = asm_valid and (not self.big or self.valid)
    self.pm.send('chestnutState', msg)


class ChestnutAccelerator:
  name = "chestnut"

  def present(self) -> bool:
    return chestnut_present()

  def ready(self) -> bool:
    # A board with no compiled pkl cannot run the large model, and compiling it
    # is the model manager's job, not something to start inside modeld.
    return chestnut_present() and chestnut_compiled()

  def unavailable_reason(self) -> str | None:
    return None  # comma's own board; nothing a user could act on

  def prepare(self) -> None:
    os.environ['HCQDEV_WAIT_TIMEOUT_MS'] = HCQ_WAIT_TIMEOUT_MS

  def make_model_state(self, cam_w: int, cam_h: int, small=None):
    from openpilot.selfdrive.modeld.modeld import ModelState  # modeld imports us
    return ModelState(cam_w, cam_h, True)

  def make_health_publisher(self, pm, model):
    return ChestnutState(pm, model.chestnut)

  def catalog(self) -> str | None:
    return "chestnut"

  def daemon(self) -> Daemon | None:
    return None
