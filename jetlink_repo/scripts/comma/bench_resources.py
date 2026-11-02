#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Sample bench processes and VM state without work on the model frame thread.
"""
import argparse
import json
import os
import re
import signal
import time
from pathlib import Path


# the sysctls the link tunes, read off the root script so a run records
# whether they were applied
ROOT_SCRIPT = Path(__file__).resolve().parents[2] / 'jetlink' / 'comma' / 'jetlink-root.sh'


def tuned_sysctls():
  pairs = re.search(r'^VM_SYSCTLS=\((.*?)\)$', ROOT_SCRIPT.read_text(), re.M | re.S).group(1).split()
  return [pair.split('=')[0] for pair in pairs]


def read(path):
  try:
    return path.read_text()
  except (OSError, UnicodeError):
    return ''


def counters(text):
  result = {}
  for line in text.splitlines():
    fields = line.replace(':', '').split()
    if len(fields) >= 2 and fields[1].isdigit():
      result[fields[0]] = int(fields[1])
  return result


def task(path):
  # comm can contain spaces and ')'; fields after the final ')' start at 3.
  fields = read(path / 'stat').rpartition(')')[2].split()
  if len(fields) < 22:
    return None
  status = counters(read(path / 'status'))
  return {'user_ticks': int(fields[11]), 'system_ticks': int(fields[12]), 'minor_faults': int(fields[7]),
          'major_faults': int(fields[9]), 'start_ticks': int(fields[19]), 'rss_pages': int(fields[21]),
          'voluntary_switches': status.get('voluntary_ctxt_switches', 0),
          'involuntary_switches': status.get('nonvoluntary_ctxt_switches', 0),
          'schedstat': read(path / 'schedstat').strip()}


def sample(pids, proc=Path('/proc'), pss=False):
  vm = counters(read(proc / 'vmstat'))
  result = {'monotonic': time.monotonic(), 'meminfo_kb': counters(read(proc / 'meminfo')),
            'vmstat': {k: v for k, v in vm.items() if k.startswith(('allocstall', 'compact_', 'pgscan_', 'pgsteal_', 'nr_dirty', 'nr_writeback'))},
            'buddyinfo': read(proc / 'buddyinfo'), 'processes': {}}
  for pid in pids:
    path = proc / str(pid)
    stats = task(path)
    if stats is None:
      continue
    stats['threads'] = {p.name: task(p) for p in (path / 'task').glob('[0-9]*')}
    if pss:
      maps = read(path / 'smaps')   # AGNOS's 4.9 has no smaps_rollup
      stats['pss_kb'] = sum(int(line.split()[1]) for line in maps.splitlines() if line.startswith('Pss:')) if maps else None
    result['processes'][str(pid)] = stats
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('pids', type=int, nargs='+')
  args = parser.parse_args()
  stop = False

  def finish(*_):
    nonlocal stop
    stop = True

  signal.signal(signal.SIGINT, finish)
  signal.signal(signal.SIGTERM, finish)
  with args.output.open('w') as output:
    metadata = {'clock_ticks': os.sysconf('SC_CLK_TCK'), 'page_bytes': os.sysconf('SC_PAGE_SIZE'),
                'commands': {str(pid): read(Path('/proc') / str(pid) / 'cmdline').replace('\0', ' ').strip() for pid in args.pids},
                'sysctls': {k: read(Path('/proc/sys') / k.replace('.', '/')).strip() for k in tuned_sysctls()}}
    output.write(json.dumps(metadata) + '\n')
    tick = 0
    while not stop:
      started = time.monotonic()
      data = sample(args.pids, pss=tick % 10 == 0)
      data['sampler_ms'] = (time.monotonic() - started) * 1000
      output.write(json.dumps(data) + '\n')
      output.flush()
      tick += 1
      time.sleep(max(0, 1 - (time.monotonic() - started)))


if __name__ == '__main__':
  main()
