#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Offroad USB byte-integrity bench. The host needs libusb1 and exclusive USB access.

This replaces inference with a digest reply; it does not measure model latency.
Use the recording bench separately. Stop the host's Jetlink server for this test
and restart it afterward. Never run the gadget side alongside modeld.
"""
import argparse
import hashlib
import json
import os
import signal
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from jetlink import protocol as P


def stamp(payload, seq):
  # Every chunk carries its frame and position, so replaying any chunk from
  # another frame differs even when the underlying random image is unchanged.
  for offset in range(0, len(payload) - 7, P.GADGET_TX_ALIGN):
    struct.pack_into('<II', payload, offset, seq, offset)


def host(args):
  import usb1

  with usb1.USBContext() as context:
    deadline = time.monotonic() + 45
    handle = None
    while handle is None and time.monotonic() < deadline:
      handle = context.openByVendorIDAndProductID(P.USB_VID, P.USB_PID, skip_on_error=True)
      if handle is None:
        time.sleep(.2)
    if handle is None:
      raise RuntimeError('no gadget enumerated within 45 seconds')
    interface = None
    try:
      setting = next(s for s in handle.getDevice().iterSettings()
                     if (s.getClass(), s.getSubClass(), s.getProtocol()) == P.USB_VENDOR_CLASS)
      endpoints = [ep.getAddress() for ep in setting.iterEndpoints()]
      ep_in = next(ep for ep in endpoints if ep & 0x80)
      ep_out = next(ep for ep in endpoints if not ep & 0x80)
      handle.claimInterface(setting.getNumber())
      interface = setting.getNumber()
      for seq in range(1, args.frames + 1):
        data = bytearray(handle.bulkRead(ep_in, P.GADGET_TX_ALIGN, timeout=30000))
        _, _, kind, received_seq, _, length, _ = P.unpack_header(data[:P.HEADER_SIZE])
        if kind != P.Msg.PING or received_seq != seq or length != args.payload_bytes:
          raise RuntimeError(f'unexpected header at frame {seq}: type={kind}, seq={received_seq}, bytes={length}')
        total = (P.HEADER_SIZE + length + P.GADGET_TX_ALIGN - 1) // P.GADGET_TX_ALIGN * P.GADGET_TX_ALIGN
        while len(data) < total:
          data.extend(handle.bulkRead(ep_in, min(total - len(data), 512 << 10), timeout=5000))
        if len(data) != total or any(data[P.HEADER_SIZE + length:]):
          raise RuntimeError(f'bad message boundary/padding at frame {seq}')
        digest = hashlib.sha256(memoryview(data)[P.HEADER_SIZE:P.HEADER_SIZE + length]).digest()
        reply = P.pack_header(P.Msg.PONG, seq, len(digest)) + digest
        if handle.bulkWrite(ep_out, reply, timeout=5000) != len(reply):
          raise RuntimeError('short digest reply')
        if seq % 1000 == 0:
          print(f'host frames={seq}', flush=True)
    finally:
      try:
        if interface is not None:
          handle.releaseInterface(interface)
      finally:
        handle.close()


def gadget(args):
  from openpilot.common.params import Params

  from jetlink.comma import lending
  from jetlink.transport.ffs import FfsTransport

  live = Params()
  if not live.get_bool('IsOffroad'):
    raise RuntimeError('USB integrity bench requires the device to remain offroad')
  for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:
      argv = path.read_bytes().split(b'\0')
    except OSError:
      continue
    if any(b'openpilot.selfdrive.modeld.modeld' in arg for arg in argv):
      raise RuntimeError('modeld is already running')
  loan = lending.borrow('usb-integrity', timeout=15)
  if loan is None:
    raise RuntimeError('could not borrow the gadget')
  transport = None
  stop = threading.Event()
  signals = 0

  def handled(*_):
    nonlocal signals
    signals += 1

  previous = signal.signal(signal.SIGUSR2, handled)
  tid = threading.get_ident()

  def interrupt_writes():
    while not stop.wait(.003):
      signal.pthread_kill(tid, signal.SIGUSR2)

  worker = threading.Thread(target=interrupt_writes)
  try:
    if loan.cable:
      raise RuntimeError('select USB for this bench')
    worker.start()
    transport = FfsTransport.borrowed(loan.mount, loan.udc, bounce=loan.bounce)
    if args.write_chunk is not None:
      transport.write_chunk = args.write_chunk
    payload = bytearray(os.urandom(args.payload_bytes))
    started = time.monotonic()
    for seq in range(1, args.frames + 1):
      if seq % 20 == 1 and not live.get_bool('IsOffroad'):
        raise RuntimeError('real ignition changed: stopping USB bench')
      stamp(payload, seq)
      expected = hashlib.sha256(payload).digest()
      transport.send(P.Msg.PING, seq, (payload,), timeout=30 if seq == 1 else .2)
      reply = transport.recv(timeout=5)
      if reply.msg_type != P.Msg.PONG or reply.seq != seq or bytes(reply.payload) != expected:
        raise RuntimeError(f'USB byte mismatch at frame {seq}: reply type={reply.msg_type} seq={reply.seq}, '
                           f'digest={bytes(reply.payload).hex()}, expected={expected.hex()}')
      if seq % 1000 == 0 or seq == args.frames:
        print(json.dumps({'verified_frames': seq, 'seconds': time.monotonic() - started, 'signals': signals}), flush=True)
  finally:
    stop.set()
    if worker.ident is not None:
      worker.join()
    signal.signal(signal.SIGUSR2, previous)
    try:
      if transport is not None:
        transport.close()
    finally:
      loan.close()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--host', action='store_true')
  parser.add_argument('--frames', type=int, default=100000)
  parser.add_argument('--payload-bytes', type=int, default=393216)
  parser.add_argument('--write-chunk', type=int, choices=[8192, 16384],
                      help='FunctionFS AIO request size; it must divide the 16 KB padding')
  args = parser.parse_args()
  if not 1 <= args.frames < 2**32 or not 8 <= args.payload_bytes <= 4 << 20:
    parser.error('frames must fit a positive uint32; payload must be between 8 bytes and 4 MiB')
  (host if args.host else gadget)(args)


if __name__ == '__main__':
  main()
