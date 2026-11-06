"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable

from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost


class SetSpeedAssistGatedBoost(AccelBoost):
  """Upstream's gas boost (and its aggressive minimum) learns past the model from gas presses. Set
  Speed Assist's floor already lifts the model, so the boost only runs with the floor off."""
  def __init__(self, assist_enabled: Callable[[], bool]):
    super().__init__()
    self.assist_enabled = assist_enabled

  def update(self, sm, output_a_target_e2e, output_a_target_mpc, a_cruise):
    if self.assist_enabled():
      self.total_boost, self.boost_eligible = 0., False
      return output_a_target_e2e
    return super().update(sm, output_a_target_e2e, output_a_target_mpc, a_cruise)
