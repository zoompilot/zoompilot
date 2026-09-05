#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import sys, collections, statistics
census = collections.Counter()
viols = []
engs = []
nobtn = []
errs = []
for line in open(sys.argv[1]):
  p = line.split()
  if not p: continue
  if p[0] == "CENSUS":
    census[p[2]] += int(p[3])
  elif p[0] == "VIOL":
    viols.append((p[1].split("test_data/")[-1], float(p[2]), p[3], int(p[4])))
  elif p[0] == "ENG":
    engs.append((p[1].split("test_data/")[-1], float(p[2]), int(p[3]), float(p[4]), p[5]))
  elif p[0] == "NOBTN":
    nobtn.append((p[1].split("test_data/")[-1], float(p[2])))
  elif p[0] in ("SKIP", "ERR"):
    errs.append(line.strip())
print("=== 3b: 0x21d byte0 census (src 2, all rlogs) ===")
tot = sum(census.values())
for v, n in sorted(census.items()):
  print(f"  {v}: {n} ({100*n/tot:.4f}%)")
print(f"total frames={tot}, non-0x7f episodes={len(viols)}")
for v in sorted(viols, key=lambda x: -x[3])[:40]:
  print("  VIOL", v)
print()
print("=== 3c: engagements (per-file scan, acc baseline per file) ===")
print(f"engagements with press tracked: {len(engs)}; rising edges with no prior press in file: {len(nobtn)}")
gaps = [e[2] for e in engs]
if gaps:
  h = collections.Counter(gaps)
  print(f"btn-frame gap: min={min(gaps)} max={max(gaps)} median={statistics.median(gaps)}")
  print(f"hist={dict(sorted(h.items()))}")
  wall = [e[3] for e in engs]
  print(f"wall press->engage: min={min(wall):.3f}s max={max(wall):.3f}s")
  over = [e for e in engs if e[2] > 10]
  print(f"gap >10 btn frames (outside mazda.h window): {len(over)}")
  for e in over: print("  OVER", e)
for e in nobtn: print("  NOBTN", e)
if errs:
  print("errors:", len(errs))
  for e in errs[:10]: print(" ", e)
