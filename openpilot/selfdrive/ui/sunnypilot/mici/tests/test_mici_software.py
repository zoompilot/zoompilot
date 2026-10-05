"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The mici software panel: a downloaded update is announced, not offered for download again.

import os
from unittest import mock

import pytest

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")


@pytest.fixture(scope="module")
def gui():
  import pyray as rl
  from openpilot.common.prefix import OpenpilotPrefix

  with OpenpilotPrefix():
    rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
    from openpilot.system.ui.lib.application import gui_app
    gui_app.init_window("test_mici_software", fps=30)
    yield gui_app
    gui_app.close()


@pytest.fixture
def updater_params(params):
  # the shared conftest's fresh params, with the updater idle and an update available remotely
  params.put("UpdaterState", "idle", block=True)
  params.put_bool("UpdaterFetchAvailable", True, block=True)
  params.put_bool("UpdateAvailable", False, block=True)
  params.put("UpdateFailedCount", 0, block=True)
  return params


def render(widget):
  import pyray as rl
  widget.render(rl.Rectangle(0, 0, 536, 240))


def test_the_panel_swaps_in_the_button_and_the_branch_button_follows(gui, updater_params):
  from openpilot.selfdrive.ui.mici.layouts.settings.software import CheckUpdateButton, TargetBranchButton
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.software import CheckUpdateButtonSP, SoftwareLayoutSP
  layout = SoftwareLayoutSP()
  items = layout._scroller.items
  checks = [i for i in items if isinstance(i, CheckUpdateButton)]
  assert [type(c) for c in checks] == [CheckUpdateButtonSP], "one check button, ours"
  branch = next(i for i in items if isinstance(i, TargetBranchButton))
  assert branch._check_update_btn is checks[0]
  assert checks[0]._touch_valid_callback is not None, "added through the scroller, so its touch gate is wrapped"


def test_a_downloaded_update_reads_ready_and_a_tap_downloads_again(gui, updater_params):
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.software import CheckUpdateButtonSP
  btn = CheckUpdateButtonSP()
  render(btn)
  assert btn.get_value() == "download update", "upstream's wording while nothing is staged"
  updater_params.put_bool("UpdateAvailable", True, block=True)
  render(btn)
  render(btn)
  assert btn.get_value() == CheckUpdateButtonSP.READY
  # a tap goes through upstream's handler, which reads the value to pick the download signal
  signalled = []
  btn._signal_updater = lambda sig: signalled.append(sig)
  with mock.patch("openpilot.selfdrive.ui.mici.layouts.settings.software.system_time_valid", return_value=True):
    btn._handle_mouse_release(None)
  assert signalled == [CheckUpdateButtonSP.DOWNLOAD_UPDATE]
