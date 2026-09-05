"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Keep these tests out of the device's real params.

Several of them exercise code whose job is to write params - jetlinkd
recording an engine as ready, or dropping that record when it finds the
Jetson's cache gone. Run on a PC that is nobody's car, and it does not matter.
Run on the comma, which CLAUDE.md tells you to do because test_queues only
passes there, and the writes land in the live params directory: a suite run
cleared JetlinkEngineReady on 2026-09-04 and left the device unable to use the
accelerator at all, because backend.ready() is params-only by design and the
record only comes back from a provision with the Jetson attached. Nothing
reported it; modeld simply loaded the small model and said nothing.

params.cc reads OPENPILOT_PREFIX on every Params() construction, so setting it
here - before pytest imports a single test module - is enough to give the whole
subtree its own directory.
"""
from openpilot.common.prefix import OpenpilotPrefix

# A prefix isolates Params and msgq. Setting the environment alone left the
# msgq directory absent on AGNOS, breaking any test using a real SubMaster.
_prefix = OpenpilotPrefix()
_prefix.__enter__()


def pytest_sessionfinish(session, exitstatus):
  _prefix.__exit__(None, None, None)
