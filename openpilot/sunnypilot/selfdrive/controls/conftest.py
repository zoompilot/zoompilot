"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix


@pytest.fixture
def params():
  """Params under a throwaway prefix, for tests that build a real controller."""
  with OpenpilotPrefix():
    yield Params()
