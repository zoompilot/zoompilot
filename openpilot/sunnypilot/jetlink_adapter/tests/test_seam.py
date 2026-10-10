"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Where jetlink meets modeld, in both modelds: stock modeld and sunnypilot's
modeld_tinygrad, which a custom small bundle runs on.

The joining model is held to whatever the loop reads and writes on the model
it runs: a read the joining model cannot answer, or a write it would keep to
itself, is found here rather than on the frame thread of a drive. This reads
the modelds rather than importing them (that costs tinygrad, usb1 and a vision
stream).
"""
import ast
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from jetlink.openpilot.joining import JoiningModelState

from openpilot.common.test import OpenpilotTestCase

OPENPILOT = Path(__file__).resolve().parents[3]


def _main(src: str) -> ast.FunctionDef:
  return next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == 'main')


def _model_attributes(src: str) -> tuple[set[str], set[str]]:
  """What main() reads and writes on `model`, the model the loop runs: every
  `model.<attr>` load and store, and getattr(model, '<attr>', ...)."""
  loads, stores = set(), set()
  for n in ast.walk(_main(src)):
    if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == 'model':
      (stores if isinstance(n.ctx, ast.Store) else loads).add(n.attr)
    elif (isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'getattr' and len(n.args) >= 2
          and isinstance(n.args[0], ast.Name) and n.args[0].id == 'model' and isinstance(n.args[1], ast.Constant)):
      loads.add(n.args[1].value)
  return loads, stores


class JoiningContract:
  PATH: Path

  def test_the_joining_model_answers_every_read_and_keeps_every_write(self):
    # read out of main() rather than kept by hand. A read the joining model
    # cannot answer is an AttributeError on the frame thread, which modeld
    # re-raises: no driving model for the drive, on jetlink devices only. A
    # write without a setter lands on the joining model and never reaches the
    # model that is driving
    loads, stores = _model_attributes(self.PATH.read_text())
    self.assertIn('run', loads, "this no longer finds what the loop reads off the model")
    self.assertEqual(sorted(n for n in loads | stores if n.startswith('_')), [])
    for name in stores:
      prop = getattr(JoiningModelState, name, None)
      self.assertTrue(isinstance(prop, property) and prop.fset is not None,
                      f"{self.PATH.name} writes model.{name}, which JoiningModelState keeps to itself")

    small = SimpleNamespace(**dict.fromkeys(loads | stores, 0.0))
    stop = threading.Event()

    def connect(should_stop=None):
      stop.wait(5)
      raise ConnectionError("no link in this test")

    joining = JoiningModelState(small, connect, build=None, progress=mock.MagicMock(), log=mock.MagicMock())
    try:
      missing = []
      for name in sorted(loads):
        try:
          getattr(joining, name)
        except AttributeError:
          missing.append(name)
      self.assertEqual(missing, [], f"JoiningModelState cannot answer {missing}, which {self.PATH.name} reads")
      for name in stores:
        setattr(joining, name, 1.0)
        self.assertEqual(getattr(small, name), 1.0, f"model.{name} = ... does not reach the small model")
    finally:
      stop.set()
      joining.close()


class StockModeld(JoiningContract, OpenpilotTestCase):
  PATH = OPENPILOT / 'selfdrive' / 'modeld' / 'modeld.py'


class ModeldTinygrad(JoiningContract, OpenpilotTestCase):
  PATH = OPENPILOT / 'sunnypilot' / 'modeld_v2' / 'modeld.py'


if __name__ == '__main__':
  unittest.main()
