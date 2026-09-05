"""Route 135: sweep the curvature-bias correction and check what it costs.

The apex score alone is not enough to accept a change that makes the planner see more
curvature than the model reports: it also has to not brake for corners that are not there.
Scores every apex, every straight stretch, and the whole engaged drive, and repeats the
whole thing against a pessimistic and an optimistic MRCC plant.
"""
import os, pickle, argparse
import numpy as np

import scc_sim_135 as sim
from scc_sim_135 import BASE, BIAS_D, BIAS_G, MPH, DT, load, prep, run, score

MIN_V = sim.MIN_V


def cfg_with(**kw):
    c = dict(BASE)
    c.update(kw)
    return c


def gain(cap=None, scale=1.0):
    g = [1.0 + (x - 1.0) * scale for x in BIAS_G]
    if cap is not None:
        g = [min(x, cap) for x in g]
    return dict(kappa_gain_d=BIAS_D, kappa_gain=g)


def straight_cost(cfg, d, s, k_real, paths, map_v, apexes, n=14):
    """Simulate stretches with no apex in them; any speed given up here is a false brake."""
    near = np.zeros(len(s), dtype=bool)
    for i in apexes:
        near[max(0, i - 400):min(len(s), i + 200)] = True
    ok = d['eng'] & ~near & (d['v'] > 12)
    runs = []
    i = 0
    while i < len(ok):
        if ok[i]:
            j = i
            while j < len(ok) and ok[j]:
                j += 1
            if (j - i) * DT >= 12:
                runs.append((i, j))
            i = j
        else:
            i += 1
    runs = runs[:n]
    lost, dur, braked = 0., 0., 0
    for i, j in runs:
        set_mph = float(np.round(np.max(d['set'][max(0, i - 200):j]) * MPH))
        r = run(cfg, d, s, k_real, paths, i, j, set_mph, map_v)
        if len(r['t']) < 5:
            continue
        real_t = (j - i) * DT
        lost += r['t'][-1] - real_t
        dur += real_t
        braked += int(r['src'].any())
    return lost, dur, braked, len(runs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plant', default='fit', choices=('fit', 'weak', 'strong'))
    args = ap.parse_args()

    if args.plant == 'weak':      # ECU 20% less capable than fitted
        sim.PLANT_A = sim.PLANT_A * np.where(sim.PLANT_A < 0, 0.8, 1.0)
    elif args.plant == 'strong':  # 20% more
        sim.PLANT_A = sim.PLANT_A * np.where(sim.PLANT_A < 0, 1.2, 1.0)

    d, mp, eps = load()
    s, k_real, paths = prep(d, mp)
    eps = [e for e in eps if e['lo'] > 5]
    map_v = np.where(np.isin(d['mapState'], ('turning',)), d['mapVT'], 1e3)
    apexes = [e['i'] for e in eps]

    cfgs = {
        'baseline (shipped)': cfg_with(),
        'gain full': cfg_with(**gain()),
        'gain cap 1.8': cfg_with(**gain(cap=1.8)),
        'gain cap 1.5': cfg_with(**gain(cap=1.5)),
        'gain cap 1.3': cfg_with(**gain(cap=1.3)),
        'gain 50% cap 1.5': cfg_with(**gain(cap=1.5, scale=0.5)),
        'plan_margin 0.85': cfg_with(plan_margin=0.85),
        'plan_margin 0.85 + gain 1.5': cfg_with(plan_margin=0.85, **gain(cap=1.5)),
        'overshoot max_gap 14': cfg_with(max_gap=14.),
        'overshoot deeper map': cfg_with(overshoot_gap=[2.0, 4.0, 6.0, 8.5, 10.0], max_gap=12.),
        'gain 1.5 + deeper map': cfg_with(overshoot_gap=[2.0, 4.0, 6.0, 8.5, 10.0], max_gap=12.,
                                          **gain(cap=1.5)),
        'walk_rate honest 4.0->3.2': cfg_with(walk_rate=3.2),
        'no dip gate': cfg_with(dip_gate=False),
        'gain 1.5, no dip gate': cfg_with(dip_gate=False, **gain(cap=1.5)),
    }

    lat_real = np.array([e['lat_apex'] for e in eps])
    print(f"plant: {args.plant}\n")
    print(f"{'AS FLOWN':30} med {np.median(lat_real):.2f} p90 {np.percentile(lat_real,90):.2f} "
          f"max {lat_real.max():.2f}  >2.0 {(lat_real>2.0).sum():2} >2.2 {(lat_real>2.2).sum():2}\n")
    print(f"{'config':30} {'med':>5} {'p90':>5} {'max':>5} {'>2.0':>5} {'>2.2':>5} "
          f"{'apexT':>7} {'strT':>7} {'strBrk':>7}")
    print('-' * 92)
    for name, cfg in cfgs.items():
        rows = score(cfg, d, s, k_real, paths, eps, map_v)
        lat = np.array([r['lat_sim'] for r in rows])
        dt = sum(r['dt'] for r in rows)
        lost, dur, braked, nrun = straight_cost(cfg, d, s, k_real, paths, map_v, apexes)
        print(f"{name:30} {np.median(lat):5.2f} {np.percentile(lat,90):5.2f} {lat.max():5.2f} "
              f"{(lat>2.0).sum():5} {(lat>2.2).sum():5} {dt:+7.1f} {lost:+7.1f} "
              f"{braked:3}/{nrun:<3}")
    print(f"\n  apexT  = seconds added over {len(eps)} curve run-ins (negative = faster than the drive)")
    print(f"  strT   = seconds added over {nrun} straight stretches ({dur:.0f}s of driving)")
    print(f"  strBrk = straight stretches where the limiter fired at all (false brakes)")


if __name__ == '__main__':
    main()
