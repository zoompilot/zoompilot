"""Route 135: where the deceleration chain loses time.

Chain on the stock path:
  planner aTarget -> ICBM overshoot gap -> dash walk (~4 mph/s) -> MRCC decel

Measures each link: the request, the commanded gap, the gap actually on the dash, and the
decel the ECU produced, plus the latency between them.
"""
import os, sys, pickle
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694
DT = 0.05


def load():
    z = np.load(os.path.join(DIR, 'resampled.npz'), allow_pickle=True)
    d = {k: z[k] for k in z.files}
    with open(os.path.join(DIR, 'episodes.pkl'), 'rb') as f:
        eps = pickle.load(f)
    return d, eps


def smooth(x, n=9):
    return np.convolve(x, np.ones(n) / n, mode='same')


def main():
    d, eps = load()
    v, setv, eng = d['v'], d['set'], d['eng']
    a_meas = smooth(np.gradient(v, DT), 11)  # measured longitudinal accel from vEgo
    gap = (v - setv) * MPH  # mph the dash sits BELOW actual speed
    limiter = np.isin(d['src'], ('sccVision', 'sccMap', 'speedLimitAssist'))

    # 1. the ECU response curve as flown on this route
    print("=== MRCC response: achieved decel vs (vEgo - dash) gap ===")
    print("   (engaged, no lead, no pedals, steady-ish gap)")
    ok = eng & ~d['lead'] & ~d['gas'] & ~d['brake'] & (v > 8)
    # require the gap to have been roughly stable for 1 s so the ECU has responded
    gap_stable = np.ones_like(gap, dtype=bool)
    for k in range(1, 21):
        gap_stable[k:] &= np.abs(gap[k:] - gap[:-k]) < 1.5
    ok &= gap_stable
    print(f"  {ok.sum()} qualifying frames")
    print(f"  {'gap mph':>10} {'n':>6} {'a med':>7} {'a p25':>7} {'a p75':>7}")
    edges = [-2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 16]
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = ok & (gap >= lo) & (gap < hi)
        if m.sum() < 20:
            continue
        print(f"  {lo:4.0f}-{hi:<5.0f} {m.sum():6} {np.median(a_meas[m]):7.2f} "
              f"{np.percentile(a_meas[m],25):7.2f} {np.percentile(a_meas[m],75):7.2f}")

    # what the shipped table predicts
    from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
        DECEL_OVERSHOOT_PARAMS)
    p = DECEL_OVERSHOOT_PARAMS['mazda']
    print(f"\n  shipped inverse map (gap needed for a decel):")
    for a, g in zip(p['decel_bp'], p['gap_v']):
        print(f"    want {a:5.2f} m/s2 -> command {g:4.1f} mph gap")

    # 2. request vs delivery during limiter-owned decel
    print("\n=== during SCC/limiter-owned frames: request vs delivery ===")
    m = eng & limiter & (v > 8) & (d['aT'] < -0.15)
    print(f"  {m.sum()} frames with a decel request")
    print(f"  requested aTarget: median {np.median(d['aT'][m]):.2f}  p10 {np.percentile(d['aT'][m],10):.2f}")
    print(f"  delivered accel:   median {np.median(a_meas[m]):.2f}  p10 {np.percentile(a_meas[m],10):.2f}")
    print(f"  commanded gap:     median {np.median(gap[m]):.1f} mph  p90 {np.percentile(gap[m],90):.1f}")
    short = a_meas[m] > d['aT'][m] + 0.1
    print(f"  frames delivering LESS decel than asked: {short.sum()} ({short.mean()*100:.0f}%)")
    print(f"  median shortfall: {np.median((a_meas[m] - d['aT'][m])[short]):.2f} m/s2")

    # 3. dash tracking: does the servo reach the commanded target?
    print("\n=== servo walk rate achieved (decreasing state) ===")
    dec = d['icbm'] == 'decreasing'
    ds = np.diff(setv * MPH)
    runs = []
    i = 0
    while i < len(dec):
        if dec[i] and eng[i]:
            j = i
            while j < len(dec) and dec[j]:
                j += 1
            if (j - i) * DT > 0.5:
                drop = (setv[i] - setv[min(j, len(setv) - 1)]) * MPH
                runs.append(((j - i) * DT, drop))
            i = j
        else:
            i += 1
    if runs:
        durs = np.array([r[0] for r in runs]); drops = np.array([r[1] for r in runs])
        rate = drops / np.maximum(durs, 1e-3)
        print(f"  {len(runs)} decreasing runs, total {durs.sum():.0f}s, total drop {drops.sum():.0f} mph")
        print(f"  achieved rate: median {np.median(rate):.2f} mph/s  p25 {np.percentile(rate,25):.2f} "
              f"p75 {np.percentile(rate,75):.2f}")
        print(f"  (limits.py assumes _SERVO_WALK_RATE mazda = 4.0 mph/s)")
        long_runs = [r for r in runs if r[0] > 2]
        if long_runs:
            lr_rate = np.array([r[1] / r[0] for r in long_runs])
            print(f"  runs > 2 s (n={len(long_runs)}): median {np.median(lr_rate):.2f} mph/s")

    # 4. per-episode chain breakdown for the hot apexes
    print("\n=== hot apexes: chain breakdown ===")
    hot = sorted([e for e in eps if e['lat_apex'] > 1.95], key=lambda e: -e['lat_apex'])
    for e in hot:
        i, lo = e['i'], e['lo']
        print(f"\n--- t={e['t']:.1f}s  latApex={e['lat_apex']:.2f}  vApex={e['v_apex']*MPH:.1f} "
              f"allowed={e['v_allowed']*MPH:.1f} mph ---")
        print(f"  {'t-apex':>7} {'v':>5} {'set':>5} {'gap':>5} {'aT':>6} {'aMeas':>6} "
              f"{'vT':>5} {'vAhd':>5} {'src':>10} {'icbm':>10} {'btn':>13} {'scc':>9}")
        for k in range(lo, min(len(v), i + 40), 4):
            print(f"  {(k-i)*DT:7.2f} {v[k]*MPH:5.1f} {setv[k]*MPH:5.1f} {gap[k]:5.1f} "
                  f"{d['aT'][k]:6.2f} {a_meas[k]:6.2f} {d['vT'][k]*MPH:5.1f} "
                  f"{min(d['vAhead'][k]*MPH,999):5.1f} {str(d['src'][k])[:10]:>10} "
                  f"{str(d['icbm'][k])[:10]:>10} {str(d['btn'][k])[:13]:>13} "
                  f"{str(d['sccState'][k])[:9]:>9}")


if __name__ == '__main__':
    main()
