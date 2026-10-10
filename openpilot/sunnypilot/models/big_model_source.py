"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The big driving model, for a chestnut build.

zoompilot's .lfsconfig leaves the big model out of every install fetch: most
devices never run it, and it is the bulk of the LFS payload. A chestnut build
fetches that one object first, as a sunnypilot install would already have.
"""
import subprocess
from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.modeld.helpers import MODELS_DIR, modeld_pkl_path

BIG_PKL = modeld_pkl_path(chestnut=True)
LFS_POINTER = b'version https://git-lfs.github.com/spec/v1'
FETCH_TIMEOUT = 3600


def is_lfs_pointer(path: Path) -> bool:
  try:
    with open(path, 'rb') as f:
      return f.read(len(LFS_POINTER)) == LFS_POINTER
  except OSError:
    return False


def _drop_big_warps(models_dir: Path) -> None:
  # chestnut_compiled() takes a pointer for the model; without the big warps it reads false,
  # so modeld stays on the small model and the uncompiled alert says why
  for warp in models_dir.glob('big_driving_warp_*_tinygrad.pkl'):
    warp.unlink(missing_ok=True)


def ensure_big_pkl(path: Path = BIG_PKL, repo: str = BASEDIR, models_dir: Path = MODELS_DIR) -> bool:
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
    _drop_big_warps(models_dir)
    return False
  if is_lfs_pointer(path):
    print(f"{path.name} is still an LFS pointer, skipping big model build", flush=True)
    _drop_big_warps(models_dir)
    return False
  return True
