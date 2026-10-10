"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import shutil
import subprocess
import tempfile
from pathlib import Path

from openpilot.common.hardware import HARDWARE

BG_JPG_PATH = Path("/usr/comma/bg.jpg")
BG_PNG_PATH = Path("/usr/comma/bg.png")
MAGIC_PATH = Path("/usr/comma/magic.py")
USER_LOGOS_DIR = Path("/data/media/bootlogos")
STOCK_BACKUP_PATH = USER_LOGOS_DIR / ".stock_backup.jpg"
LOGO_EXTENSIONS = (".jpg", ".jpeg", ".png")
MAX_USER_LOGO_SIZE = 10 * 1024 * 1024  # bytes
CANVAS_SIZE = (2160, 1080)  # the screen; magic.py draws bg.jpg scaled to width x width/2
LOGO_BOX_SIZE = (256, 440)  # footprint of the stock comma logo, measured from stock bg.jpg


def _run(cmd: list[str], timeout: float = 30) -> subprocess.CompletedProcess:
  try:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
  except subprocess.TimeoutExpired:
    return subprocess.CompletedProcess(cmd, 124, stdout="", stderr="timeout")


def _first_user_logo() -> Path | None:
  """First user-supplied logo by file name (ASCII order); None when the folder holds none."""
  try:
    entries = sorted(USER_LOGOS_DIR.iterdir())
  except OSError:
    return None

  for entry in entries:
    try:
      if not entry.is_file() or entry.name.startswith(".") or entry.suffix.lower() not in LOGO_EXTENSIONS:
        continue
      if entry.stat().st_size > MAX_USER_LOGO_SIZE:
        continue
    except OSError:
      continue
    return entry
  return None


def _ensure_user_logos_dir() -> None:
  """Create the drop folder up front so the first copy needs no manual mkdir. Comma-owned 755."""
  try:
    USER_LOGOS_DIR.mkdir(parents=True, exist_ok=True)
  except OSError as error:
    print(f"boot logo: failed to create {USER_LOGOS_DIR}: {error}")


def _backup_stock_logo() -> None:
  """Keep the device's own stock image so an empty folder can restore it. Reads / only, writes /data."""
  if STOCK_BACKUP_PATH.is_file() or not BG_JPG_PATH.is_file():
    return
  try:
    shutil.copyfile(BG_JPG_PATH, STOCK_BACKUP_PATH)
  except OSError as error:
    print(f"boot logo: failed to back up stock logo: {error}")


def _stage_canvas(logo):
  """Flatten onto black, fit into the stock logo's box, center on the boot canvas."""
  from PIL import Image

  flattened = Image.alpha_composite(Image.new("RGBA", logo.size, (0, 0, 0, 255)), logo.convert("RGBA")).convert("RGB")
  scale = min(LOGO_BOX_SIZE[0] / flattened.width, LOGO_BOX_SIZE[1] / flattened.height, 1.0)
  resized = flattened.resize((max(1, round(flattened.width * scale)), max(1, round(flattened.height * scale))), Image.LANCZOS)
  canvas = Image.new("RGB", CANVAS_SIZE, (0, 0, 0))
  canvas.paste(resized, ((CANVAS_SIZE[0] - resized.width) // 2, (CANVAS_SIZE[1] - resized.height) // 2))
  return canvas


def _write_logo(target: Path) -> None:
  from PIL import Image

  with tempfile.TemporaryDirectory(prefix="boot_logo_") as staging_dir:
    staging_path = Path(staging_dir)
    staged_jpeg = staging_path / "bg.jpg"
    staged_png = staging_path / "bg.png"

    # current AGNOS draws bg.jpg through magic.py; older generations drew bg.png through weston
    magic_uses_jpeg = False
    try:
      magic_uses_jpeg = BG_JPG_PATH.as_posix() in MAGIC_PATH.read_text()
    except OSError:
      pass

    if target == STOCK_BACKUP_PATH:
      # the backup was a valid bg.jpg: restore it byte-for-byte, no re-encode
      shutil.copyfile(target, staged_jpeg)
      with Image.open(target) as img:
        img.convert("RGB").save(staged_png, format="PNG")
    else:
      with Image.open(target) as img:
        landscape = _stage_canvas(img)
      landscape.save(staged_png, format="PNG")
      (landscape if magic_uses_jpeg else landscape.transpose(Image.Transpose.ROTATE_270)).save(staged_jpeg, format="JPEG", quality=95)

    variants = [(staged_jpeg, BG_JPG_PATH)]
    if BG_PNG_PATH.is_file():
      variants.append((staged_png, BG_PNG_PATH))

    pending = [(source, destination) for source, destination in variants
               if not destination.is_file() or destination.read_bytes() != source.read_bytes()]
    if not pending:
      return

    mount = _run(["findmnt", "-n", "-o", "OPTIONS", "/"])
    if mount.returncode != 0:
      print(f"boot logo: failed to read mount options: {mount.stderr.strip()}")
      return
    mount_options = mount.stdout.strip()

    remount_rw = _run(["sudo", "mount", "-o", "remount,rw", "/"])
    if remount_rw.returncode != 0:
      print(f"boot logo: failed to remount / read-write: {remount_rw.stderr.strip()}")
      return

    try:
      for source, destination in pending:
        copied = _run(["sudo", "cp", str(source), str(destination)])
        if copied.returncode != 0:
          print(f"boot logo: failed to write {destination}: {copied.stderr.strip()}")
    finally:
      restored = _run(["sudo", "mount", "-o", f"remount,{mount_options}", "/"])
      if restored.returncode != 0:
        print(f"boot logo: failed to restore mount options: {restored.stderr.strip()}")


def apply_boot_logo() -> None:
  """Apply the first logo in /data/media/bootlogos, or restore the stock logo when the folder is empty.

  Selection is folder state only: no param, no UI. The drop folder is created when missing.
  Never raises: the caller runs at every manager start and a failure must not block boot.
  """
  try:
    if HARDWARE.get_device_type() == "pc":
      return

    _ensure_user_logos_dir()

    target = _first_user_logo()
    if target is not None:
      _backup_stock_logo()
      _write_logo(target)
    elif STOCK_BACKUP_PATH.is_file():
      _write_logo(STOCK_BACKUP_PATH)
  except Exception as error:
    print(f"boot logo: apply failed: {error}")
