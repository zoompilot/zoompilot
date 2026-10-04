import capnp
import dataclasses
import functools
from typing import Any

from openpilot.cereal import custom
from opendbc.car import structs

_FIELDS = '__dataclass_fields__'  # copy of dataclasses._FIELDS


def is_dataclass(obj):
  """Similar to dataclasses.is_dataclass without instance type check checking"""
  return hasattr(obj, _FIELDS)


def _asdictref_inner(obj) -> dict[str, Any] | Any:
  if is_dataclass(obj):
    ret = {}
    for field in getattr(obj, _FIELDS):  # similar to dataclasses.fields()
      ret[field] = _asdictref_inner(getattr(obj, field))
    return ret
  elif isinstance(obj, (tuple, list)):
    return type(obj)(_asdictref_inner(v) for v in obj)
  else:
    return obj


def asdictref(obj) -> dict[str, Any]:
  """
  Similar to dataclasses.asdict without recursive type checking and copy.deepcopy
  Note that the resulting dict will contain references to the original struct as a result
  """
  if not is_dataclass(obj):
    raise TypeError("asdictref() should be called on dataclass instances")

  return _asdictref_inner(obj)


def convert_to_capnp(struct: structs.CarParamsSP | structs.CarStateSP) -> capnp.lib.capnp._DynamicStructBuilder:
  struct_dict = asdictref(struct)

  if isinstance(struct, structs.CarParamsSP):
    struct_capnp = custom.CarParamsSP.new_message(**struct_dict)
  elif isinstance(struct, structs.CarStateSP):
    struct_capnp = custom.CarStateSP.new_message(**struct_dict)
  else:
    raise ValueError(f"Unsupported struct type: {type(struct)}")

  return struct_capnp


# CarControlSP's sub-structs, converted from their dicts; params stays as to_dict gives it
CAR_CONTROL_SP_SUBSTRUCTS = {
  'mads': structs.ModularAssistiveDrivingSystem,
  'leadOne': structs.LeadData,
  'leadTwo': structs.LeadData,
  'intelligentCruiseButtonManagement': structs.IntelligentCruiseButtonManagement,
}


@functools.cache
def _field_names(cls) -> frozenset[str]:
  return frozenset(f.name for f in dataclasses.fields(cls))


def _build(cls, s: dict):
  # only the dataclass's own fields: a capnp-only field (deprecated, or one opendbc does not
  # carry, like mads.lateralHeld) would otherwise be an unexpected kwarg and crash card
  names = _field_names(cls)
  return cls(**{k: v for k, v in s.items() if k in names})


def convert_carControlSP(struct: capnp.lib.capnp._DynamicStructReader) -> structs.CarControlSP:
  struct_dict = struct.to_dict()
  substructs = {k: _build(cls, struct_dict.get(k, {})) for k, cls in CAR_CONTROL_SP_SUBSTRUCTS.items()}
  return _build(structs.CarControlSP, {**struct_dict, **substructs})
