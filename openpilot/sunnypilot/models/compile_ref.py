"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import argparse
import hashlib
import os

from openpilot.common.basedir import BASEDIR

# the scripts that write the driving and dm warp pkls, repo-relative. one list for scons
# (modeld/SConscript rebuilds on them) and for CI (the prebuilt workflow keys the HF defaults
# on them next to tinygrad_ref), so a pkl cannot outlive the script that wrote it.
# nothing heavier than basedir is imported here: the CI resolve jobs run it on a bare runner
MODELD_COMPILE_SCRIPTS = (
  'openpilot/selfdrive/modeld/compile_modeld.py',
  'openpilot/selfdrive/modeld/get_model_metadata.py',
  'openpilot/system/camerad/cameras/nv12_info.py',
  'openpilot/common/hardware/hw.py',
)
DM_WARP_COMPILE_SCRIPTS = ('openpilot/selfdrive/modeld/compile_dm_warp.py',)
COMPILE_SCRIPTS = MODELD_COMPILE_SCRIPTS + DM_WARP_COMPILE_SCRIPTS


def git_blob_sha(data: bytes) -> str:
  # what `git rev-parse HEAD:<path>` and the GitHub contents API report for the file
  return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def compile_ref_from_blob_shas(shas: list[str]) -> str:
  return hashlib.sha256("".join(f"{s}\n" for s in shas).encode()).hexdigest()


def get_compile_ref() -> str:
  shas = []
  for path in COMPILE_SCRIPTS:
    with open(os.path.join(BASEDIR, path), "rb") as f:
      shas.append(git_blob_sha(f.read()))
  return compile_ref_from_blob_shas(shas)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--list", action="store_true", help="print the script paths instead of the ref")
  args = parser.parse_args()
  print("\n".join(COMPILE_SCRIPTS) if args.list else get_compile_ref())


if __name__ == "__main__":
  main()
