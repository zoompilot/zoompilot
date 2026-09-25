"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The big driving model's ONNX, for a chestnut build.

zoompilot's .lfsconfig leaves the big model out of every install fetch: most
devices never run it, and it is the bulk of the LFS payload. A chestnut
compiles it at build time, so a chestnut build fetches that one object first,
as a sunnypilot install would already have.
"""
import subprocess
from pathlib import Path

from openpilot.common.basedir import BASEDIR

BIG_ONNX = Path(BASEDIR) / 'openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx'
LFS_POINTER = b'version https://git-lfs.github.com/spec/v1'
FETCH_TIMEOUT = 3600


def is_lfs_pointer(path: Path) -> bool:
  try:
    with open(path, 'rb') as f:
      return f.read(len(LFS_POINTER)) == LFS_POINTER
  except OSError:
    return False


def ensure_big_onnx(path: Path = BIG_ONNX, repo: str = BASEDIR) -> bool:
  """True once `path` is the model itself, fetching it if it is still a pointer.

  False, with the reason printed, when the fetch fails (offline, say): the
  build then goes on without the big model rather than failing whole, and the
  next build tries again.
  """
  if not is_lfs_pointer(path):
    return path.is_file()
  print(f"Fetching {path.name} for the chestnut; installs leave it out (.lfsconfig)", flush=True)
  try:
    # -X "" drops the fetchexclude for this one pull
    subprocess.run(['git', 'lfs', 'pull', '--include', str(path.relative_to(repo)), '--exclude', ''],
                   cwd=repo, check=True, timeout=FETCH_TIMEOUT)
  except (OSError, subprocess.SubprocessError) as e:
    print(f"Could not fetch {path.name}, skipping big model build: {e}", flush=True)
    return False
  if is_lfs_pointer(path):
    print(f"{path.name} is still an LFS pointer, skipping big model build", flush=True)
    return False
  return True
