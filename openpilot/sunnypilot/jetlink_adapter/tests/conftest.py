"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

jetlink from this checkout's jetlink_repo, ahead of anything installed: the
path a device gets from the jetlink symlink launch_chffrplus.sh makes.
"""
import sys
from pathlib import Path

JETLINK_REPO = str(Path(__file__).resolve().parents[4] / 'jetlink_repo')
if JETLINK_REPO in sys.path:
  sys.path.remove(JETLINK_REPO)
sys.path.insert(0, JETLINK_REPO)
