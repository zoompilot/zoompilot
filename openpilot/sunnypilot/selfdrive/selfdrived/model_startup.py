"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

modeld's first load, held off like a big model's.

On a comma 4 modeld publishes its first frame 10-20 s after it starts (the
tinygrad import, then the model), past selfdrived's 6 s initialization timeout,
so every boot raised commIssue and locationdTemporaryError for everything
downstream of the model. Until the model and what follows it are up, the
native bigModelLoading holds engagement off instead, and those checks wait as
they do for a big model load (selfdrived's big_model_settling).

It runs once from initialization and never comes back, so a modeld that dies
mid-drive is still a commIssue, and it is bounded, so one that never comes up
is too.
"""
from openpilot.cereal import messaging
from openpilot.common.realtime import DT_CTRL

# after the first frame, for the services downstream of the model to follow it
SETTLE_S = 5.
# from selfdrived's start; the slowest boot in the drive logs had its first frame at 24 s
TIMEOUT_S = 30.


class ModelStartup:
  def __init__(self):
    self.starting = True
    self.deadline = round(TIMEOUT_S / DT_CTRL)  # selfdrived frame

  def update(self, sm: messaging.SubMaster) -> bool:
    """True while modeld's first load holds engagement off."""
    if self.starting:
      if sm.seen['modelV2']:
        self.deadline = min(self.deadline, sm.frame + round(SETTLE_S / DT_CTRL))
      # deviceMotion.inputsOK is locationd's own verdict, not one all_checks covers
      inputs_up = (sm.seen['modelV2'] and (not sm.seen['deviceMotion'] or sm['deviceMotion'].inputsOK)
                   and sm.all_checks())
      self.starting = not inputs_up and sm.frame <= self.deadline
    return self.starting
