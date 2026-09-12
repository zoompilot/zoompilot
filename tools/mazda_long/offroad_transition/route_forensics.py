#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Timestamped event table for a Mazda alpha-long route: radar takeover / hand-back, cruise
arming, software stops and ignition, from raw CAN plus the published state.

Usage (on device):
  cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python \
    /tmp/route_forensics.py /data/media/0/realdata 0000020e [--segments 0,1] [--from 100 --to 200]

Route time is seconds since the first logged message of the lowest segment given.
Only edges are printed for periodic signals; counts per segment are summarised at the end.
No VIN or dongle id is printed.

can src semantics (panda): src 0/1/2 = received on that bus; src 128+bus = a frame the panda
put on that bus itself, which is BOTH openpilot's own sendcan traffic and the panda's forwarded
copies of the other side (bus 0 -> 2 and 2 -> 0 once the safety model is not elm327), so
"copy src130" of CRZ_BTNS is the wheel's frame relayed to the camera, not ours; src 192+bus =
a frame the panda refused (rejection report). Use sendcan to attribute transmissions to
openpilot (sendcan_census.py).
"""
import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader

DBC = "/data/openpilot/opendbc_repo/opendbc/dbc/mazda_2017.dbc"

# message: (address, {signal: (start, length, big_endian)}) read from the DBC below
WANT = {
  "PEDALS": ["ACC_OFF", "ACC_ACTIVE", "BRAKE_ON", "STANDSTILL"],
  "CRZ_BTNS": ["MODE_X", "MODE_Y", "CAN_OFF", "RES", "SET_P", "SET_M", "TJA_BUTTON", "DISTANCE_LESS", "DISTANCE_MORE", "CTR"],
  "CRZ_CTRL": ["CRZ_AVAILABLE", "CRZ_ACTIVE", "DISTANCE_SETTING", "ACC_ACTIVE_2"],
  "CRZ_INFO": ["ACC_ACTIVE", "ACC_SET_ALLOWED", "NEW_SIGNAL_7", "ACCEL_CMD", "CTR"],
  "CRZ_EVENTS": ["CRZ_SPEED"],
  "CAM_LANEINFO": ["NO_ERR_BIT", "ERR_BIT", "BIT2", "LANE_LINES", "TJA"],
  "STEER_RATE": ["LKAS_BLOCK", "LKAS_FAULT", "LKAS_TRACK_STATE"],
  "CAM_LKAS": ["ERR_BIT_1", "ERR_BIT_2", "LKAS_REQUEST"],
  "RADAR_UDS_RESPONSE": ["PCI", "SID", "SUB", "NRC"],
  "GEAR": ["BRAKE_HOLD"],
  "ENGINE_DATA": ["SPEED"],
}
RADAR_TX = 0x764
RADAR_RX = 0x76c
LOG_TERMS = ("mazdaRadar", "hand-back", "handback", "OffroadMode", "onroad", "Onroad", "alpha long",
             "Startup", "shutting", "reboot", "Reboot", "restoration", "stock ECU", "timed out",
             "ignition", "Ignition", "panda", "Panda", "sendcan", "Controls Mismatch", "mismatch")
LOG_SKIP = ("STATUS_PACKET", "chestnut", "Interrupt", "NACK", "usb", "spi", "PowerMonitoring",
            "network", "modem", "GPS", "gps", "comm_interface", "uploader", "athena", "sunnylink",
            "logMessage", "ThermalStatus", "thermal", "screen", "brightness", "dmonitoring",
            "modeld", "camerad", "encoder", "loggerd", "bootlog", "deleter", "timezone")


def load_dbc():
  msgs = {}
  cur = None
  for line in Path(DBC).read_text().splitlines():
    m = re.match(r"BO_ (\d+) (\w+):", line)
    if m:
      cur = m.group(2)
      msgs[cur] = (int(m.group(1)), {})
      continue
    m = re.match(r"\s*SG_ (\w+) : (\d+)\|(\d+)@([01])([+-]) \(([^,]+),([^)]+)\)", line)
    if m and cur in WANT and m.group(1) in WANT[cur]:
      msgs[cur][1][m.group(1)] = (int(m.group(2)), int(m.group(3)), m.group(4) == "0", float(m.group(6)), float(m.group(7)))
  return {name: msgs[name] for name in WANT}


def decode(sigs, dat):
  out = {}
  n = len(dat) * 8
  be = int.from_bytes(dat, "big")
  le = int.from_bytes(dat, "little")
  for name, (start, length, big, scale, offset) in sigs.items():
    mask = (1 << length) - 1
    if big:
      pos = (start // 8) * 8 + (7 - start % 8)   # MSB position counted from the frame's MSB
      raw = (be >> (n - pos - length)) & mask
    else:
      raw = (le >> start) & mask
    out[name] = raw * scale + offset if (scale != 1 or offset != 0) else raw
  return out


class Tracker:
  def __init__(self, t_from, t_to):
    self.events = []
    self.last = {}
    self.t_from, self.t_to = t_from, t_to
    self.counts = Counter()
    self.last_seen = {}
    self.silence_open = {}
    # distinct CRZ_CTRL payloads by source: payload -> [count, first_t, last_t]
    self.ctrl_payloads = defaultdict(lambda: defaultdict(lambda: [0, None, None]))

  def payload(self, t, source_key, dat):
    rec = self.ctrl_payloads[source_key][dat.hex()]
    rec[0] += 1
    rec[1] = t if rec[1] is None else rec[1]
    rec[2] = t

  def emit(self, t, source, text):
    if self.t_from <= t <= self.t_to:
      self.events.append((t, source, text))

  def edge(self, t, key, value, source, fmt=None):
    prev = self.last.get(key)
    self.last[key] = value
    if prev != value:
      self.emit(t, source, f"{key} {prev} -> {value}" if fmt is None else fmt(prev, value))
      return True
    return False

  def presence(self, t, key, gap, source):
    """Report silence longer than gap and the return of a periodic message."""
    prev = self.last_seen.get(key)
    self.last_seen[key] = t
    if prev is None:
      self.emit(t, source, f"{key} first seen")
    elif self.silence_open.get(key):
      self.emit(t, source, f"{key} resumed after {t - prev:.2f} s silence")
      self.silence_open[key] = False

  def close_silences(self, t, keys_gaps, source):
    for key, gap in keys_gaps.items():
      prev = self.last_seen.get(key)
      if prev is not None and not self.silence_open.get(key) and t - prev > gap:
        self.silence_open[key] = True
        self.emit(prev, source, f"{key} silent from here (last frame)")


def scan(root, route, segments, t_from, t_to, all_buttons):
  dbc = load_dbc()
  by_addr = {addr: (name, sigs) for name, (addr, sigs) in dbc.items()}
  seg_dirs = sorted(Path(root).glob(f"{route}--*--*"), key=lambda p: int(p.name.rsplit("--", 1)[1]))
  if segments is not None:
    seg_dirs = [p for p in seg_dirs if int(p.name.rsplit("--", 1)[1]) in segments]
  tr = Tracker(t_from, t_to)
  t0 = None
  seg_summary = {}
  for seg in seg_dirs:
    seg_idx = int(seg.name.rsplit("--", 1)[1])
    rlog = seg / "rlog.zst"
    if not rlog.exists():
      rlog = seg / "qlog.zst"
    counts = Counter()
    last_can_t = None
    for msg in LogReader(str(rlog)):
      if t0 is None:
        t0 = msg.logMonoTime
      t = (msg.logMonoTime - t0) / 1e9
      kind = msg.which()
      if kind == "initData":
        seg_summary[seg_idx] = {"commit": msg.initData.gitCommit[:10], "branch": msg.initData.gitBranch}
        tr.emit(t, "initData", f"seg {seg_idx} starts, commit {msg.initData.gitCommit[:10]} branch {msg.initData.gitBranch}")
      elif kind == "carParams":
        cp = msg.carParams
        tr.emit(t, "carParams", f"opLong={cp.openpilotLongitudinalControl} alphaAvail={cp.alphaLongitudinalAvailable} " +
                                 f"fp={cp.carFingerprint} safetyParam={cp.safetyConfigs[0].safetyParam if len(cp.safetyConfigs) else None}")
      elif kind == "pandaStates":
        if len(msg.pandaStates):
          p = msg.pandaStates[0]
          up_prev = tr.last.get("panda.uptime")
          tr.last["panda.uptime"] = p.uptime
          if up_prev is not None and p.uptime < up_prev:
            tr.emit(t, "pandaStates", f"panda uptime RESET {up_prev} -> {p.uptime}")
          if "panda.first" not in tr.last:
            tr.last["panda.first"] = True
            tr.emit(t, "pandaStates", f"first pandaStates uptime={p.uptime} ignLine={p.ignitionLine} ignCan={p.ignitionCan} " +
                                       f"safety={p.safetyModel} controlsAllowed={p.controlsAllowed}")
          tr.edge(t, "panda.ignitionLine", p.ignitionLine, "pandaStates")
          tr.edge(t, "panda.ignitionCan", p.ignitionCan, "pandaStates")
          tr.edge(t, "panda.safetyModel", str(p.safetyModel), "pandaStates")
          tr.edge(t, "panda.controlsAllowed", p.controlsAllowed, "pandaStates")
          tr.edge(t, "panda.harness", str(p.harnessStatus), "pandaStates")
      elif kind == "deviceState":
        tr.edge(t, "deviceState.started", msg.deviceState.started, "deviceState")
      elif kind == "logMessage":
        try:
          entry = json.loads(msg.logMessage)
        except ValueError:
          continue
        if not isinstance(entry, dict):
          continue
        text = str(entry.get("msg", ""))
        if text.startswith("VIN "):
          continue
        ctx = entry.get("ctx", {}) or {}
        daemon = ctx.get("daemon", "?")
        if any(s in text for s in LOG_SKIP) and not any(s in text for s in ("mazdaRadar", "hand-back", "OffroadMode")):
          continue
        if any(s in text for s in LOG_TERMS) or daemon in ("card", "hardwared", "manager"):
          if len(text) > 400:
            text = text[:400] + "..."
          tr.emit(t, f"log:{daemon}:{entry.get('levelnum', '')}", text)
      elif kind == "carState":
        cs = msg.carState
        tr.edge(t, "carState.cruiseState.available", cs.cruiseState.available, "carState")
        tr.edge(t, "carState.cruiseState.enabled", cs.cruiseState.enabled, "carState")
        tr.edge(t, "carState.accFaulted", cs.accFaulted, "carState")
        tr.edge(t, "carState.canValid", cs.canValid, "carState")
        tr.edge(t, "carState.standstill", cs.standstill, "carState")
        tr.edge(t, "carState.steerFaultPermanent", cs.steerFaultPermanent, "carState")
        for be in cs.buttonEvents:
          tr.emit(t, "carState.buttonEvents", f"{be.type} pressed={be.pressed}")
        tr.last["vEgo"] = cs.vEgo
      elif kind == "carControl":
        cc = msg.carControl
        tr.edge(t, "carControl.enabled", cc.enabled, "carControl")
        tr.edge(t, "carControl.latActive", cc.latActive, "carControl")
        tr.edge(t, "carControl.longActive", cc.longActive, "carControl")
      # carControlSP.stockEcuHandBack is card-side only and never published; the hand-back is
      # visible through the mazdaRadarSession carlog lines and the 0x764 default-session frames.
      elif kind == "selfdriveState":
        tr.edge(t, "selfdriveState.enabled", msg.selfdriveState.enabled, "selfdriveState")
        tr.edge(t, "selfdriveState.state", str(msg.selfdriveState.state), "selfdriveState")
      elif kind == "sendcan":
        for f in msg.sendcan:
          counts[f"sendcan:{f.src}:{f.address:x}"] += 1
          if f.address == RADAR_TX:
            tr.emit(t, "sendcan", f"UDS tx 0x764 bus{f.src} {bytes(f.dat).hex()}")
          if f.address in (0x21b, 0x21c):
            tr.presence(t, f"sendcan.{by_addr[f.address][0]}.bus{f.src}", 0.1, "sendcan")
            if f.address == 0x21c:
              v = decode(dbc["CRZ_CTRL"][1], bytes(f.dat))
              tr.edge(t, f"sendcan.CRZ_CTRL.bus{f.src}.AVAIL/ACTIVE", (v["CRZ_AVAILABLE"], v["CRZ_ACTIVE"]), "sendcan")
            else:
              v = decode(dbc["CRZ_INFO"][1], bytes(f.dat))
              tr.edge(t, f"sendcan.CRZ_INFO.bus{f.src}.ACTIVE/SET_ALLOWED/NS7", (v["ACC_ACTIVE"], v["ACC_SET_ALLOWED"], v["NEW_SIGNAL_7"]), "sendcan")
      elif kind == "can":
        for f in msg.can:
          src, addr = f.src, f.address
          dat = bytes(f.dat)
          if src in (0, 2):
            last_can_t = t
          if addr == RADAR_TX or addr == RADAR_RX:
            counts[f"can:{src}:{addr:x}"] += 1
            tr.emit(t, "can", f"UDS {'tx' if addr == RADAR_TX else 'rx'} 0x{addr:x} src{src} {dat.hex()}")
            continue
          if src == 192:
            counts[f"can:192:{addr:x}"] += 1
            continue
          if src >= 128:
            counts[f"can:{src}:{addr:x}"] += 1
            if addr == 0x21c:
              tr.payload(t, f"echo src{src}", dat)
            if addr in (0x21b, 0x21c, 0x9d):
              tr.presence(t, f"echo.{by_addr[addr][0]}.src{src}", 0.1, "can")
              if addr == 0x9d:
                v = decode(dbc["CRZ_BTNS"][1], dat)
                tr.emit(t, "can", f"CRZ_BTNS copy src{src} {dat.hex()} " + " ".join(f"{k}={v[k]}"
                          for k in ("CAN_OFF", "RES", "SET_P", "SET_M", "TJA_BUTTON", "CTR")))
            continue
          if addr not in by_addr:
            continue
          name, sigs = by_addr[addr]
          counts[f"can:{src}:{name}"] += 1
          v = decode(sigs, dat)
          key = f"{name}.src{src}"
          if name == "PEDALS":
            tr.presence(t, key, 0.2, "can")
            tr.edge(t, key + ".ACC_OFF(armed)/ACC_ACTIVE/BRAKE_ON/STANDSTILL", (v["ACC_OFF"], v["ACC_ACTIVE"], v["BRAKE_ON"], v["STANDSTILL"]), "can")
          elif name == "CRZ_BTNS":
            main = int(v["MODE_X"] == 1 and v["MODE_Y"] == 1)
            tr.edge(t, key + ".MAIN(MODE_X&MODE_Y)", main, "can", lambda p, n, d=dat: f"CRZ_BTNS MAIN {p} -> {n} raw {d.hex()}")
            tr.edge(t, key + ".MODE_X/MODE_Y", (v["MODE_X"], v["MODE_Y"]), "can")
            if all_buttons:
              for b in ("CAN_OFF", "RES", "SET_P", "SET_M", "TJA_BUTTON", "DISTANCE_LESS", "DISTANCE_MORE"):
                tr.edge(t, f"{key}.{b}", v[b], "can")
            else:
              for b in ("CAN_OFF", "RES", "TJA_BUTTON"):
                tr.edge(t, f"{key}.{b}", v[b], "can")
          elif name == "CRZ_CTRL":
            tr.presence(t, key, 0.1, "can")
            tr.payload(t, f"can src{src}", dat)
            tr.edge(t, key + ".AVAIL/ACTIVE/GAP/ACTIVE2", (v["CRZ_AVAILABLE"], v["CRZ_ACTIVE"], v["DISTANCE_SETTING"], v["ACC_ACTIVE_2"]), "can")
          elif name == "CRZ_INFO":
            tr.presence(t, key, 0.1, "can")
            tr.edge(t, key + ".ACTIVE/SET_ALLOWED/NS7", (v["ACC_ACTIVE"], v["ACC_SET_ALLOWED"], v["NEW_SIGNAL_7"]), "can")
          elif name == "CAM_LANEINFO":
            tr.presence(t, key, 1.5, "can")
            tr.edge(t, key + ".NO_ERR/ERR/BIT2/LANES/TJA", (v["NO_ERR_BIT"], v["ERR_BIT"], v["BIT2"], v["LANE_LINES"], v["TJA"]), "can")
          elif name == "STEER_RATE":
            tr.edge(t, key + ".BLOCK/FAULT/TRACK", (v["LKAS_BLOCK"], v["LKAS_FAULT"], v["LKAS_TRACK_STATE"]), "can")
          elif name == "CAM_LKAS":
            tr.presence(t, key, 0.2, "can")
            tr.edge(t, key + ".ERR1/ERR2", (v["ERR_BIT_1"], v["ERR_BIT_2"]), "can")
          elif name == "RADAR_UDS_RESPONSE":
            pass
          elif name == "GEAR":
            tr.edge(t, key + ".BRAKE_HOLD", v["BRAKE_HOLD"], "can")
          elif name == "ENGINE_DATA":
            tr.presence(t, key, 0.1, "can")
            tr.last["speed_kph"] = v["SPEED"]
            tr.edge(t, "ENGINE_DATA.moving(>0.1kph)", v["SPEED"] > 0.1, "can")
          elif name == "CRZ_EVENTS":
            tr.edge(t, key + ".CRZ_SPEED", round(v["CRZ_SPEED"]), "can")
        if last_can_t is not None:
          tr.close_silences(t, {"PEDALS.src0": 0.2, "CRZ_INFO.src0": 0.1, "CRZ_CTRL.src0": 0.1, "ENGINE_DATA.src0": 0.1,
                                "CAM_LANEINFO.src2": 1.5, "CAM_LKAS.src2": 0.2,
                                "sendcan.CRZ_INFO.bus0": 0.1, "sendcan.CRZ_CTRL.bus0": 0.1,
                                "echo.CRZ_INFO.src128": 0.1, "echo.CRZ_CTRL.src128": 0.1}, "can")
    seg_summary.setdefault(seg_idx, {})["counts"] = dict(counts)
    seg_summary[seg_idx]["t_end"] = round(t, 2)
    seg_summary[seg_idx]["last_speed_kph"] = tr.last.get("speed_kph")
  return tr, seg_summary


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("root")
  ap.add_argument("route")
  ap.add_argument("--segments", default=None, help="comma list of segment indices")
  ap.add_argument("--from", dest="t_from", type=float, default=-1.)
  ap.add_argument("--to", dest="t_to", type=float, default=1e9)
  ap.add_argument("--all-buttons", action="store_true")
  ap.add_argument("--json", action="store_true")
  args = ap.parse_args()
  segments = None if args.segments is None else {int(s) for s in args.segments.split(",")}
  tr, summary = scan(args.root, args.route, segments, args.t_from, args.t_to, args.all_buttons)
  tr.events.sort(key=lambda e: e[0])
  if args.json:
    print(json.dumps({"events": tr.events, "segments": summary}, indent=1, default=str))
    return
  for t, source, text in tr.events:
    print(f"{t:9.3f}  {source:34s} {text}")
  print("\n== distinct CRZ_CTRL (0x21c) payloads by source: hex count first_t last_t ==")
  for src_key in sorted(tr.ctrl_payloads):
    for hx, (n, ft, lt) in sorted(tr.ctrl_payloads[src_key].items(), key=lambda kv: kv[1][1]):
      print(f"  {src_key:14s} {hx} n={n:6d} {ft:9.3f} .. {lt:9.3f}")
  print("\n== per-segment counts ==")
  for idx in sorted(summary):
    s = summary[idx]
    keys = ("can:0:CRZ_INFO", "can:0:CRZ_CTRL", "can:0:PEDALS", "can:0:CRZ_BTNS", "can:128:21b", "can:128:21c", "can:130:21b",
            "sendcan:0:21b", "sendcan:0:21c", "sendcan:0:764", "sendcan:0:499", "can:0:764", "can:128:764", "can:0:76c",
            "can:192:243", "can:128:243", "can:2:CAM_LKAS", "can:2:CAM_LANEINFO", "can:0:RADAR_UDS_RESPONSE")
    c = s.get("counts", {})
    print(f"seg {idx}: t_end={s.get('t_end')} commit={s.get('commit')} last_speed_kph={s.get('last_speed_kph')} " +
          " ".join(f"{k.split(':',1)[1]}={c.get(k, 0)}" for k in keys if c.get(k, 0)))


if __name__ == "__main__":
  main()
