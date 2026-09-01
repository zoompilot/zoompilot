"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params


class ModelStateBase:
  def __init__(self):
    self.lat_delay = Params().get("LagdValueCache", return_default=True)

  def make_chestnut_state(self, pm):
    """The accelerator health publisher for this model.

    Asking the model rather than testing its type is what keeps modeld from
    growing a branch per accelerator: a jetlink Jetson publishes the same
    chestnutState from its own telemetry by overriding this.
    """
    from openpilot.selfdrive.modeld.modeld import ChestnutState  # modeld imports us
    return ChestnutState(pm, self.chestnut)
