"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.cereal import custom
from openpilot.selfdrive.car.helpers import convert_carControlSP


class TestConvertCarControlSP:
  def test_capnp_only_substructs_do_not_crash(self):
    """capnp CarControlSP can carry substructs that have no field on the opendbc dataclass;
    the converter must drop them instead of passing them as kwargs."""
    msg = custom.CarControlSP.new_message()
    msg.mads.enabled = True
    msg.leadOne.dRel = 12.5
    struct_dict = {**msg.as_reader().to_dict(), 'capnpOnlySubstruct': {'gain': 0.5}}

    out = convert_carControlSP(SimpleNamespace(to_dict=lambda: struct_dict))
    assert out.mads.enabled is True
    assert abs(out.leadOne.dRel - 12.5) < 1e-6
