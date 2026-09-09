#!/usr/bin/env python3
import hashlib
import json
import lzma
import os
import struct
import subprocess
import time
from collections.abc import Generator

import requests

SPARSE_CHUNK_FMT = struct.Struct('H2xI4x')

AGNOS_MANIFEST_FILE = "openpilot/system/hardware/comma/agnos.json"
AGNOS_MANIFEST_FILE_TICI = "openpilot/system/hardware/comma/tici_agnos.json"
ABCTL = "abctl"
ACTIVATION_ATTEMPTS = 3
ACTIVATION_RETRY_DELAY = 1


def get_device_model() -> str:
  try:
    with open("/sys/firmware/devicetree/base/model") as f:
      return f.read().strip("\x00").strip().split("comma ")[-1]
  except OSError:
    return ""


def get_manifest_file() -> str:
  """The comma three needs its own boot image; every other partition is comma's."""
  return AGNOS_MANIFEST_FILE_TICI if get_device_model() == "tici" else AGNOS_MANIFEST_FILE



class StreamingDecompressor:
  def __init__(self, url: str) -> None:
    self.buf = b""

    self.req = requests.get(url, stream=True, headers={'Accept-Encoding': 'identity'}, timeout=60)
    self.it = self.req.iter_content(chunk_size=1024 * 1024)
    self.decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_AUTO)
    self.eof = False
    self.sha256 = hashlib.sha256()

  def read(self, length: int) -> bytes:
    while len(self.buf) < length and not self.eof:
      if self.decompressor.needs_input:
        self.req.raise_for_status()

        try:
          compressed = next(self.it)
        except StopIteration:
          self.eof = True
          break
      else:
        compressed = b''

      self.buf += self.decompressor.decompress(compressed, max_length=length)

      if self.decompressor.eof:
        self.eof = True
        break

    result = self.buf[:length]
    self.buf = self.buf[length:]

    self.sha256.update(result)
    return result


def unsparsify(f: StreamingDecompressor) -> Generator[bytes, None, None]:
  # https://source.android.com/devices/bootloader/images#sparse-format
  magic = struct.unpack("I", f.read(4))[0]
  assert(magic == 0xed26ff3a)

  # Version
  major = struct.unpack("H", f.read(2))[0]
  minor = struct.unpack("H", f.read(2))[0]
  assert(major == 1 and minor == 0)

  f.read(2)  # file header size
  f.read(2)  # chunk header size

  block_sz = struct.unpack("I", f.read(4))[0]
  f.read(4)  # total blocks
  num_chunks = struct.unpack("I", f.read(4))[0]
  f.read(4)  # crc checksum

  for _ in range(num_chunks):
    chunk_type, out_blocks = SPARSE_CHUNK_FMT.unpack(f.read(12))

    if chunk_type == 0xcac1:  # Raw
      # TODO: yield in smaller chunks. Yielding only block_sz is too slow. Largest observed data chunk is 252 MB.
      yield f.read(out_blocks * block_sz)
    elif chunk_type == 0xcac2:  # Fill
      filler = f.read(4) * (block_sz // 4)
      for _ in range(out_blocks):
        yield filler
    elif chunk_type == 0xcac3:  # Don't care
      yield b""
    else:
      raise Exception("Unhandled sparse chunk type")


# noop wrapper with same API as unsparsify() for non sparse images
def noop(f: StreamingDecompressor) -> Generator[bytes, None, None]:
  while len(chunk := f.read(1024 * 1024)) > 0:
    yield chunk


def get_active_slot_number() -> int:
  current_slot = subprocess.check_output([ABCTL, "--boot_slot"], text=True).strip()
  if current_slot not in ("_a", "_b"):
    raise RuntimeError(f"unexpected active slot: {current_slot!r}")
  return 0 if current_slot == "_a" else 1


def get_target_slot_number() -> int:
  return 1 - get_active_slot_number()


def slot_number_to_suffix(slot_number: int) -> str:
  if slot_number not in (0, 1):
    raise ValueError(f"invalid slot number: {slot_number}")
  return '_a' if slot_number == 0 else '_b'


def validate_target_slot_number(target_slot_number: int) -> None:
  slot_number_to_suffix(target_slot_number)
  if target_slot_number == get_active_slot_number():
    raise RuntimeError(f"target slot {target_slot_number} is already active")


def get_partition_path(target_slot_number: int, partition: dict) -> str:
  path = f"/dev/disk/by-partlabel/{partition['name']}"

  if partition.get('has_ab', True):
    path += slot_number_to_suffix(target_slot_number)

  return path


def get_raw_hash(path: str, partition_size: int) -> str:
  raw_hash = hashlib.sha256()
  pos, chunk_size = 0, 1024 * 1024

  with open(path, 'rb') as out:
    while pos < partition_size:
      n = min(chunk_size, partition_size - pos)
      raw_hash.update(out.read(n))
      pos += n

  return raw_hash.hexdigest().lower()


def verify_partition(target_slot_number: int, partition: dict[str, str | int], force_full_check: bool = False) -> bool:
  full_check = partition['full_check'] or force_full_check
  path = get_partition_path(target_slot_number, partition)

  if not isinstance(partition['size'], int):
    return False

  partition_size: int = partition['size']

  if not isinstance(partition['hash_raw'], str):
    return False

  partition_hash: str = partition['hash_raw']

  if full_check:
    return get_raw_hash(path, partition_size) == partition_hash.lower()
  else:
    with open(path, 'rb') as out:
      out.seek(partition_size)
      return out.read(64) == partition_hash.lower().encode()


def clear_partition_hash(target_slot_number: int, partition: dict) -> None:
  path = get_partition_path(target_slot_number, partition)
  with open(path, 'wb+') as out:
    partition_size = partition['size']

    out.seek(partition_size)
    out.write(b"\x00" * 64)
    os.sync()


def load_manifest(manifest_path: str) -> list[dict]:
  with open(manifest_path) as f:
    manifest = json.load(f)
  if not isinstance(manifest, list):
    raise ValueError("AGNOS manifest must be a list")
  return manifest


def extract_compressed_image(target_slot_number: int, partition: dict, cloudlog):
  path = get_partition_path(target_slot_number, partition)
  downloader = StreamingDecompressor(partition['url'])

  with open(path, 'wb+') as out:
    # Flash partition
    last_p = 0
    raw_hash = hashlib.sha256()
    f = unsparsify if partition['sparse'] else noop
    for chunk in f(downloader):
      raw_hash.update(chunk)
      out.write(chunk)
      p = int(out.tell() / partition['size'] * 100)
      if p != last_p:
        last_p = p
        print(f"Installing {partition['name']}: {p}", flush=True)

    if raw_hash.hexdigest().lower() != partition['hash_raw'].lower():
      raise Exception(f"Raw hash mismatch '{raw_hash.hexdigest().lower()}'")

    if downloader.sha256.hexdigest().lower() != partition['hash'].lower():
      raise Exception("Uncompressed hash mismatch")

    if out.tell() != partition['size']:
      raise Exception("Uncompressed size mismatch")

    os.sync()


def check_partition_fits(target_slot_number: int, partition: dict) -> None:
  # A manifest entry larger than the partition cannot be flashed: agnos.py writes
  # the decompressed image straight at the block device and the write fails at the
  # end. Refuse before touching anything, so a bad manifest does not leave the
  # inactive slot half written, and so the reason is in the log.
  path = get_partition_path(target_slot_number, partition)
  try:
    with open(path, 'rb') as f:
      available = f.seek(0, os.SEEK_END)
  except OSError:
    return  # not a block device, e.g. running the updater off device

  if partition['size'] > available:
    raise Exception(f"{partition['name']} is {partition['size']} bytes, "
                    f"{partition['size'] - available} more than the {available} byte partition")


def flash_partition(target_slot_number: int, partition: dict, cloudlog, standalone=False):
  cloudlog.info(f"Downloading and writing {partition['name']}")

  check_partition_fits(target_slot_number, partition)

  if verify_partition(target_slot_number, partition):
    cloudlog.info(f"Already flashed {partition['name']}")
    return

  # Clear hash before flashing in case we get interrupted
  full_check = partition['full_check']
  if not full_check:
    clear_partition_hash(target_slot_number, partition)

  path = get_partition_path(target_slot_number, partition)

  extract_compressed_image(target_slot_number, partition, cloudlog)

  # Write hash after successful flash
  if not full_check:
    with open(path, 'wb+') as out:
      out.seek(partition['size'])
      out.write(partition['hash_raw'].lower().encode())


def activate_slot(target_slot_number: int, cloudlog) -> None:
  validate_target_slot_number(target_slot_number)
  for attempt in range(1, ACTIVATION_ATTEMPTS + 1):
    try:
      subprocess.run([ABCTL, "--set_active", str(target_slot_number)], check=True, capture_output=True, text=True)
      if get_active_slot_number() == target_slot_number:
        cloudlog.info(f"Activated slot {target_slot_number}")
        return
      cloudlog.error(f"Activation postcondition failed for slot {target_slot_number}")
    except subprocess.CalledProcessError as exc:
      cloudlog.error(f"Activation failed for slot {target_slot_number}: {exc}")

    if attempt < ACTIVATION_ATTEMPTS:
      time.sleep(ACTIVATION_RETRY_DELAY)

  raise RuntimeError(f"failed to activate slot {target_slot_number}")


def swap(manifest_path: str, target_slot_number: int, cloudlog) -> None:
  validate_target_slot_number(target_slot_number)
  if not verify_agnos_update(manifest_path, target_slot_number):
    raise RuntimeError(f"AGNOS verification failed for target slot {target_slot_number}")

  # Once the slot boots, its partitions are no longer what the manifest hashed.
  # Clear the trailers so the fast check cannot pass on a slot that has run;
  # a later update to it will reflash. Full-check partitions carry no trailer.
  for partition in load_manifest(manifest_path):
    if not partition.get('full_check', False):
      clear_partition_hash(target_slot_number, partition)

  activate_slot(target_slot_number, cloudlog)


def flash_agnos_update(manifest_path: str, target_slot_number: int, cloudlog, standalone=False) -> None:
  validate_target_slot_number(target_slot_number)
  update = load_manifest(manifest_path)
  if not update:
    raise ValueError("AGNOS manifest must contain at least one partition")

  cloudlog.info(f"Target slot {target_slot_number}")

  # set target slot as unbootable
  subprocess.run([ABCTL, "--set_unbootable", str(target_slot_number)], check=True)

  for partition in update:
    success = False

    for retries in range(10):
      try:
        flash_partition(target_slot_number, partition, cloudlog, standalone)
        if not verify_partition(target_slot_number, partition):
          raise RuntimeError(f"verification failed for {partition['name']}")
        success = True
        break

      except requests.exceptions.RequestException:
        cloudlog.exception("Failed")
        cloudlog.info(f"Failed to download {partition['name']}, retrying ({retries})")
        time.sleep(10)

    if not success:
      cloudlog.info(f"Failed to flash {partition['name']}, aborting")
      raise Exception("Maximum retries exceeded")

  if not verify_agnos_update(manifest_path, target_slot_number):
    raise RuntimeError(f"AGNOS verification failed for target slot {target_slot_number}")

  cloudlog.info(f"AGNOS ready on slot {target_slot_number}")


def verify_agnos_update(manifest_path: str, target_slot_number: int) -> bool:
  update = load_manifest(manifest_path)
  return bool(update) and all(verify_partition(target_slot_number, partition) for partition in update)


def main(argv: list[str] | None = None) -> int:
  import argparse
  import logging

  parser = argparse.ArgumentParser(description="Flash and verify AGNOS update",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  action = parser.add_mutually_exclusive_group()
  action.add_argument("--verify", action="store_true", help="Verify and perform swap if update ready")
  action.add_argument("--verify-only", action="store_true", help="Verify without changing slots")
  action.add_argument("--swap", action="store_true", help="Verify and perform swap, downloads if necessary")
  parser.add_argument("manifest", help="Manifest json")
  args = parser.parse_args(argv)

  logging.basicConfig(level=logging.INFO)

  target_slot_number = get_target_slot_number()
  if args.verify_only:
    return 0 if verify_agnos_update(args.manifest, target_slot_number) else 1
  elif args.verify:
    if verify_agnos_update(args.manifest, target_slot_number):
      swap(args.manifest, target_slot_number, logging)
      return 0
    return 1
  elif args.swap:
    if not verify_agnos_update(args.manifest, target_slot_number):
      logging.error("Verification failed. Flashing AGNOS")
      flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)

    if not verify_agnos_update(args.manifest, target_slot_number):
      raise RuntimeError("AGNOS verification failed after flashing")
    logging.warning(f"Verification succeeded. Swapping to slot {target_slot_number}")
    swap(args.manifest, target_slot_number, logging)
  else:
    flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
