#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Keep the Jetson awake from the comma, for a bench session.

Borrows the gadget from its owner (jetlinkd) and holds the loan. The owner
keeps a lent gadget presented, so the Jetson enumerates it, wakes if it was
asleep, and stays up until the loan goes back.

    jetlink_repo/scripts/comma/jetlink_hold.py                 until Ctrl-C or SIGTERM
    jetlink_repo/scripts/comma/jetlink_hold.py -t 600          for ten minutes
    jetlink_repo/scripts/comma/jetlink_hold.py -- CMD [ARGS]   while CMD runs; exits with its status

The owner lends to one borrower at a time, so this loan keeps modeld and a
bench (bench_link.py --loan, jetlink_live_bench.sh) out: stop it before either,
and before a provisioning run.
"""
from __future__ import annotations

import argparse
import logging
import signal
import subprocess
import sys
import time
from pathlib import Path

# the checkout this sits in, which is the jetlink the owner runs; a copy
# elsewhere takes jetlink from PYTHONPATH (/data/openpilot on a comma)
ROOT = Path(__file__).resolve().parents[2]
if (ROOT / 'jetlink' / 'comma').is_dir():
  sys.path.insert(0, str(ROOT))

from jetlink.comma import gadget, lending  # noqa: E402

BORROW_TIMEOUT = 60.0
POLL = 0.5


def say(msg: str) -> None:
  print(f'jetlink_hold: {msg}', flush=True)


def status(returncode: int) -> int:
  """A child's exit status as a shell reports it: 128 + N for signal N."""
  return returncode if returncode >= 0 else 128 - returncode


def hold(loan: lending.Loan, seconds: float | None, command: list[str]) -> int:
  child = subprocess.Popen(command) if command else None
  end = None if seconds is None else time.monotonic() + seconds
  lost = False
  try:
    while True:
      wait = POLL if end is None else max(min(POLL, end - time.monotonic()), 0)
      if child is not None:
        try:
          return status(child.wait(timeout=wait))
        except subprocess.TimeoutExpired:
          pass
      # the owner closing the loan's socket: it exited or restarted, and the
      # loan went with it. Without a child, this is the wait
      gone = not lost and lending.hung_up(loan.conn, 0 if child is not None else wait)
      if end is not None and time.monotonic() >= end:
        return 0
      if gone:
        lost = True
        say('the owner went away, and the loan with it')
        if child is None:
          return 1
  except KeyboardInterrupt:
    if child is None:
      return 0   # the usual way to end an open-ended hold
    if child.poll() is None:
      child.terminate()
      try:
        child.wait(5.0)
      except subprocess.TimeoutExpired:
        child.kill()
        child.wait()
    return status(child.returncode)


def main(argv: list[str] | None = None, path: Path = lending.SOCKET) -> int:
  argv = sys.argv[1:] if argv is None else list(argv)
  command: list[str] = []
  dashed = '--' in argv
  if dashed:
    split = argv.index('--')
    argv, command = argv[:split], argv[split + 1:]
  parser = argparse.ArgumentParser(description='Borrow the gadget from the owner and hold it, so the Jetson stays awake.',
                                   usage='%(prog)s [-t SECONDS] [--timeout SECONDS] [-- CMD [ARGS]]')
  parser.add_argument('-t', '--seconds', type=float, help='hold this long, then give the loan back')
  parser.add_argument('--timeout', type=float, default=BORROW_TIMEOUT, help='how long to wait for the loan (default %(default)g s)')
  args = parser.parse_args(argv)
  if args.seconds is not None and command:
    parser.error('give -t or a command, not both')
  if dashed and not command:
    parser.error('-- needs a command')

  # Not a probe connect first: the owner takes one connection at a time, and
  # a probe queued beside the borrow can get the borrow refused.
  started = time.monotonic()
  loan = lending.borrow('hold', timeout=args.timeout, path=path) if path.exists() else None
  if loan is None:
    if time.monotonic() - started < min(1.0, args.timeout / 2):   # refused at once, not timed out
      why = gadget.gadget_error() or 'is Jetlink on and jetlinkd running?'
      say(f'no gadget owner is listening on {path}: {why}')
    else:
      say(f'no loan from the owner within {args.timeout:g} s; is modeld or a bench holding it?')
    return 1
  try:
    link = 'the cable link' if loan.cable else 'the gadget'
    say(f'holding {link}: udc {loan.udc}, mount {loan.mount}')
    return hold(loan, args.seconds, command)
  finally:
    loan.close()
    say('released')


def _interrupt(signum, frame):
  raise KeyboardInterrupt


if __name__ == '__main__':
  # the lending layer's routine lines ("borrowed the gadget ...") repeat ours;
  # its errors still show
  gadget.log.setLevel(logging.ERROR)
  signal.signal(signal.SIGTERM, _interrupt)
  try:
    sys.exit(main())
  except KeyboardInterrupt:   # before there was a loan to hold
    sys.exit(130)
