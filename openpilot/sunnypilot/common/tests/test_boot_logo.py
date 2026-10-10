"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import pytest

import openpilot.sunnypilot.common.boot_logo as bl


def make_image(path: Path, size: tuple[int, int]):
  path.parent.mkdir(parents=True, exist_ok=True)
  Image.new("RGB", size, (12, 34, 56)).save(path)


class FakeRunner:
  def __init__(self, fail_cp_dest: str | None = None):
    self.commands: list[list[str]] = []
    self.fail_cp_dest = fail_cp_dest

  def __call__(self, cmd, **kwargs):
    self.commands.append(list(cmd))
    if cmd[0] == "findmnt":
      return subprocess.CompletedProcess(cmd, 0, stdout="ro,relatime", stderr="")
    if cmd[:2] == ["sudo", "mount"]:
      return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")
    if cmd[:2] == ["sudo", "cp"]:
      if self.fail_cp_dest is not None and cmd[3] == self.fail_cp_dest:
        return subprocess.CompletedProcess(cmd, 1, stdout="", stderr="cp: error")
      shutil.copyfile(cmd[2], cmd[3])
      return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")
    return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")


@pytest.fixture
def boot_env(monkeypatch, tmp_path) -> SimpleNamespace:
  comma_dir = tmp_path / "usr" / "comma"
  comma_dir.mkdir(parents=True)
  bg_jpg = comma_dir / "bg.jpg"
  bg_png = comma_dir / "bg.png"
  make_image(bg_jpg, (32, 16))
  make_image(bg_png, (32, 16))
  stock_jpeg_bytes = bg_jpg.read_bytes()
  stock_png_bytes = bg_png.read_bytes()

  user_dir = tmp_path / "data" / "media" / "bootlogos"

  # current AGNOS: magic.py draws bg.jpg, so staged JPEGs stay landscape
  magic = comma_dir / "magic.py"
  magic.write_text(f'BACKGROUND = "{bg_jpg.as_posix()}"\n')

  monkeypatch.setattr(bl, "BG_JPG_PATH", bg_jpg)
  monkeypatch.setattr(bl, "BG_PNG_PATH", bg_png)
  monkeypatch.setattr(bl, "MAGIC_PATH", magic)
  monkeypatch.setattr(bl, "USER_LOGOS_DIR", user_dir)
  monkeypatch.setattr(bl, "STOCK_BACKUP_PATH", user_dir / ".stock_backup.jpg")
  monkeypatch.setattr(bl.HARDWARE, "get_device_type", lambda: "tici")

  runner = FakeRunner()
  monkeypatch.setattr(bl, "_run", runner)
  return SimpleNamespace(bg_jpg=bg_jpg, bg_png=bg_png, magic=magic, user_dir=user_dir,
                         backup=user_dir / ".stock_backup.jpg", runner=runner,
                         stock_jpeg_bytes=stock_jpeg_bytes, stock_png_bytes=stock_png_bytes)


def image_size(path: Path) -> tuple[int, int]:
  with Image.open(path) as img:
    return img.size


def test_pc_device_returns_without_writes(boot_env, monkeypatch):
  monkeypatch.setattr(bl.HARDWARE, "get_device_type", lambda: "pc")
  make_image(boot_env.user_dir / "custom.jpg", (24, 12))
  bl.apply_boot_logo()
  assert boot_env.runner.commands == []
  assert boot_env.bg_jpg.read_bytes() == boot_env.stock_jpeg_bytes


def test_first_user_logo_picks_alphanumeric_first(boot_env):
  make_image(boot_env.user_dir / "b.png", (24, 12))
  make_image(boot_env.user_dir / "a.jpg", (24, 12))
  make_image(boot_env.user_dir / "B.JPG", (24, 12))
  (boot_env.user_dir / "c.txt").write_bytes(b"ignore")
  make_image(boot_env.user_dir / ".hidden.jpg", (24, 12))

  # ASCII order: uppercase names sort before lowercase; dotfiles and other suffixes are skipped
  assert bl._first_user_logo() == boot_env.user_dir / "B.JPG"


def test_oversized_logo_is_skipped(boot_env):
  big = boot_env.user_dir / "a_big.jpg"
  big.parent.mkdir(parents=True, exist_ok=True)
  with big.open("wb") as f:
    f.write(b"\0" * (bl.MAX_USER_LOGO_SIZE + 1))
  make_image(boot_env.user_dir / "b.jpg", (24, 12))

  assert bl._first_user_logo() == boot_env.user_dir / "b.jpg"


def test_missing_dir_is_created_and_stock_no_op(boot_env):
  bl.apply_boot_logo()
  assert boot_env.user_dir.is_dir()
  assert boot_env.runner.commands == []
  assert not boot_env.backup.exists()
  assert boot_env.bg_jpg.read_bytes() == boot_env.stock_jpeg_bytes


def test_uncreatable_dir_is_not_fatal(boot_env, monkeypatch):
  blocker = boot_env.user_dir.parent / "blocker"
  blocker.mkdir(parents=True, exist_ok=True)
  (blocker / "file").write_bytes(b"x")
  uncreatable = blocker / "file" / "bootlogos"
  monkeypatch.setattr(bl, "USER_LOGOS_DIR", uncreatable)
  monkeypatch.setattr(bl, "STOCK_BACKUP_PATH", uncreatable / ".stock_backup.jpg")
  bl.apply_boot_logo()
  assert boot_env.runner.commands == []
  assert boot_env.bg_jpg.read_bytes() == boot_env.stock_jpeg_bytes


def test_empty_folder_without_backup_is_no_op(boot_env):
  boot_env.user_dir.mkdir(parents=True)
  bl.apply_boot_logo()
  assert boot_env.runner.commands == []
  assert boot_env.bg_jpg.read_bytes() == boot_env.stock_jpeg_bytes


def test_user_logo_lands_on_boot_canvas(boot_env):
  make_image(boot_env.user_dir / "custom.jpg", (12, 24))
  bl.apply_boot_logo()
  assert image_size(boot_env.bg_jpg) == bl.CANVAS_SIZE
  assert image_size(boot_env.bg_png) == bl.CANVAS_SIZE


def test_portrait_canvas_for_legacy_weston(boot_env):
  boot_env.magic.write_text("# legacy weston magic, no bg.jpg\n")
  make_image(boot_env.user_dir / "custom.jpg", (24, 12))
  bl.apply_boot_logo()
  assert image_size(boot_env.bg_jpg) == (bl.CANVAS_SIZE[1], bl.CANVAS_SIZE[0])
  assert image_size(boot_env.bg_png) == bl.CANVAS_SIZE


def test_magic_sniff_failure_is_treated_as_legacy(boot_env):
  boot_env.magic.unlink()
  make_image(boot_env.user_dir / "custom.jpg", (12, 24))
  bl.apply_boot_logo()
  assert image_size(boot_env.bg_jpg) == (bl.CANVAS_SIZE[1], bl.CANVAS_SIZE[0])


def test_png_variant_only_written_when_destination_exists(boot_env):
  boot_env.bg_png.unlink()
  make_image(boot_env.user_dir / "custom.jpg", (24, 12))
  bl.apply_boot_logo()
  assert not boot_env.bg_png.exists()
  assert image_size(boot_env.bg_jpg) == bl.CANVAS_SIZE


def test_no_write_when_destination_bytes_match(boot_env):
  make_image(boot_env.user_dir / "custom.jpg", (24, 12))
  bl.apply_boot_logo()
  written = boot_env.bg_jpg.read_bytes()
  boot_env.runner.commands.clear()
  bl.apply_boot_logo()
  assert boot_env.runner.commands == []
  assert boot_env.bg_jpg.read_bytes() == written


def test_happy_path_command_order(boot_env):
  make_image(boot_env.user_dir / "custom.jpg", (24, 12))
  bl.apply_boot_logo()
  commands = boot_env.runner.commands
  assert commands[0] == ["findmnt", "-n", "-o", "OPTIONS", "/"]
  assert commands[1] == ["sudo", "mount", "-o", "remount,rw", "/"]
  cp_commands = [cmd for cmd in commands if cmd[:2] == ["sudo", "cp"]]
  assert [cmd[3] for cmd in cp_commands] == [str(boot_env.bg_jpg), str(boot_env.bg_png)]
  assert all(cmd[2].endswith(("bg.jpg", "bg.png")) for cmd in cp_commands)
  assert commands[-1] == ["sudo", "mount", "-o", "remount,ro,relatime", "/"]
  assert len(commands) == 5


def test_failed_copy_still_restores_mount_options(boot_env):
  boot_env.runner.fail_cp_dest = str(boot_env.bg_jpg)
  make_image(boot_env.user_dir / "custom.jpg", (24, 12))
  bl.apply_boot_logo()
  mount_commands = [cmd for cmd in boot_env.runner.commands if cmd[:2] == ["sudo", "mount"]]
  assert mount_commands == [
    ["sudo", "mount", "-o", "remount,rw", "/"],
    ["sudo", "mount", "-o", "remount,ro,relatime", "/"],
  ]
  assert boot_env.bg_jpg.read_bytes() == boot_env.stock_jpeg_bytes


def test_stock_backup_taken_once(boot_env):
  make_image(boot_env.user_dir / "custom.jpg", (24, 12))
  bl.apply_boot_logo()
  assert boot_env.backup.read_bytes() == boot_env.stock_jpeg_bytes

  boot_env.backup.write_bytes(b"marker")
  bl.apply_boot_logo()
  assert boot_env.backup.read_bytes() == b"marker"


def test_empty_folder_restores_stock(boot_env):
  custom = boot_env.user_dir / "custom.jpg"
  make_image(custom, (24, 12))
  bl.apply_boot_logo()
  assert image_size(boot_env.bg_jpg) == bl.CANVAS_SIZE

  custom.unlink()
  bl.apply_boot_logo()
  assert boot_env.bg_jpg.read_bytes() == boot_env.stock_jpeg_bytes


def test_transparent_png_flattens_onto_black(boot_env):
  logo_path = boot_env.user_dir / "custom.png"
  logo_path.parent.mkdir(parents=True, exist_ok=True)
  logo = Image.new("RGBA", (24, 12), (0, 255, 0, 0))  # green hidden under full transparency
  logo.paste((255, 0, 0, 255), (0, 0, 12, 12))
  logo.save(logo_path)
  bl.apply_boot_logo()

  with Image.open(boot_env.bg_jpg) as flat:
    center_x, center_y = bl.CANVAS_SIZE[0] // 2, bl.CANVAS_SIZE[1] // 2
    opaque = flat.getpixel((center_x - 4, center_y))
    transparent = flat.getpixel((center_x + 4, center_y))
    assert opaque[0] >= 250 and opaque[1] <= 5
    assert transparent[1] <= 30


def test_small_logo_is_centered_not_upscaled(boot_env):
  make_image(boot_env.user_dir / "custom.jpg", (100, 50))
  bl.apply_boot_logo()
  center_x, center_y = bl.CANVAS_SIZE[0] // 2, bl.CANVAS_SIZE[1] // 2

  with Image.open(boot_env.bg_jpg) as flat:
    center = flat.getpixel((center_x, center_y))
    assert all(abs(channel - expected) <= 2 for channel, expected in zip(center, (12, 34, 56), strict=True))
    assert flat.getpixel((center_x + 80, center_y)) == (0, 0, 0)
    assert flat.getpixel((2, 2)) == (0, 0, 0)


def test_corrupt_logo_writes_nothing(boot_env):
  garbage = boot_env.user_dir / "garbage.jpg"
  garbage.parent.mkdir(parents=True, exist_ok=True)
  garbage.write_bytes(b"not an image")
  bl.apply_boot_logo()
  assert boot_env.runner.commands == []
  assert boot_env.bg_jpg.read_bytes() == boot_env.stock_jpeg_bytes


def test_run_timeout_becomes_failure(monkeypatch):
  def hang(cmd, **kwargs):
    raise subprocess.TimeoutExpired(cmd, timeout=30)
  monkeypatch.setattr(bl.subprocess, "run", hang)
  result = bl._run(["sudo", "mount", "-o", "remount,rw", "/"])
  assert result.returncode == 124
  assert result.stderr == "timeout"
