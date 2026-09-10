import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from openpilot.common.api.backend import lock_sentinel


class FakeSubprocess:
  def __init__(self, rm_returncode, fmt_returncode):
    self.returncodes = [rm_returncode, 0, fmt_returncode, 0]
    self.commands = []

  def run(self, command, *, shell):
    assert shell
    self.commands.append(command)
    return SimpleNamespace(returncode=self.returncodes[len(self.commands) - 1])


def stub_module(monkeypatch, name, **attributes):
  module = ModuleType(name)
  for key, value in attributes.items():
    setattr(module, key, value)
  monkeypatch.setitem(sys.modules, name, module)


def load_reset_module(monkeypatch, relative_path):
  class StubWidget:
    pass

  stub_module(monkeypatch, "pyray")
  stub_module(monkeypatch, "openpilot.system.ui.lib.application",
              gui_app=SimpleNamespace(), FontWeight=SimpleNamespace(), FONT_SCALE=1)
  stub_module(monkeypatch, "openpilot.system.ui.widgets", Widget=StubWidget)
  stub_module(monkeypatch, "openpilot.system.ui.widgets.button",
              Button=StubWidget, ButtonStyle=SimpleNamespace(PRIMARY=0))
  stub_module(monkeypatch, "openpilot.system.ui.widgets.label",
              UnifiedLabel=StubWidget, gui_label=lambda *a, **k: None, gui_text_box=lambda *a, **k: None)
  stub_module(monkeypatch, "openpilot.system.ui.widgets.scroller", Scroller=StubWidget)
  stub_module(monkeypatch, "openpilot.system.ui.mici_setup",
              GreyBigButton=StubWidget, FailedPage=StubWidget)
  stub_module(monkeypatch, "openpilot.selfdrive.ui.mici.widgets.dialog",
              BigDialog=StubWidget, BigConfirmationCircleButton=StubWidget)

  path = Path(relative_path)
  name = f"factory_reset_test_{path.stem}"
  spec = importlib.util.spec_from_file_location(name, path)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


@pytest.mark.parametrize(("relative_path", "failure_attr"), [
  ("openpilot/system/ui/tici_reset.py", "_reset_state"),
  ("openpilot/system/ui/mici_reset.py", "_reset_failed"),
])
@pytest.mark.parametrize(("rm_returncode", "fmt_returncode", "reboots"), [
  (0, 1, True),
  (1, 0, True),
  (1, 1, False),
])
def test_factory_reset_erases_userdata_before_optional_reboot(monkeypatch, relative_path, failure_attr,
                                                               rm_returncode, fmt_returncode, reboots):
  module = load_reset_module(monkeypatch, relative_path)
  subprocess = FakeSubprocess(rm_returncode, fmt_returncode)
  monkeypatch.setattr(module, "PC", False)
  monkeypatch.setattr(module, "USERDATA", "/dev/userdata")
  monkeypatch.setattr(module, "subprocess", subprocess)
  reset = SimpleNamespace()

  module.Reset._do_erase(reset)

  assert subprocess.commands[:3] == [
    "sudo rm -rf /data/*",
    "sudo umount /dev/userdata",
    "yes | sudo mkfs.ext4 /dev/userdata",
  ]
  assert (subprocess.commands[-1] == "sudo reboot") is reboots
  if not reboots:
    assert hasattr(reset, failure_attr)


def test_lock_sentinel_is_inside_factory_reset_userdata():
  class ProductionParams:
    @staticmethod
    def get_param_path():
      return "/data/params/d"

  params = ProductionParams()
  sentinel = Path(params.get_param_path()).parent / ".konik_lockout"
  assert lock_sentinel(params) == sentinel == Path("/data/params/.konik_lockout")
  assert Path("/data") in sentinel.parents
