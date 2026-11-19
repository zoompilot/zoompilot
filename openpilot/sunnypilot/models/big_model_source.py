"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The built-in big driving model, which zoompilot does not ship.

zoompilot's .lfsconfig leaves big_driving_tinygrad.pkl out of every install
and update fetch (about 800 MB, and most devices never run it), so on a
zoompilot install it is a git lfs pointer. A chestnut instead runs the Default
big model as an ordinary download through the model manager
(ModelManagerSP._fetch_big_model_files), so stock modeld only has to tell a
pointer from the model.
"""
from pathlib import Path

LFS_POINTER = b'version https://git-lfs.github.com/spec/v1'


def is_lfs_pointer(path: Path) -> bool:
  try:
    with open(path, 'rb') as f:
      return f.read(len(LFS_POINTER)) == LFS_POINTER
  except OSError:
    return False
