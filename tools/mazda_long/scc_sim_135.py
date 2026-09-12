"""Route 135: closed-loop simulator for SCC + decel overshoot on the stock MRCC path.

The controller plans on path GEOMETRY (kappa vs distance), which does not change when the
car goes slower, so the recorded model paths can be replayed against a different speed
profile by indexing them on distance travelled rather than on time. That makes a real
closed-loop test possible: solver -> overshoot lever -> dash servo -> fitted MRCC plant ->
speed -> back into the solver.

Scores every curve apex the drive passed through by the lateral acceleration the simulated
car would have pulled, plus what the change costs in journey time.
"""
import os, pickle, json, argparse
import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import (
    allowed_speed, backward_pass, lead_distance, min_profile_speed, required_decel)

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694
DT = 0.05
V_FLOOR = 0.5
MIN_V = 20 / 3.6

# MRCC plant, fitted from this route (scc_plant_135.py) + the corpus map shape
PLANT_GAP = np.array([-14., -10., -7., -5., -3., -1.5, 0., 1.5, 2.5, 4., 6., 8., 10., 14.])
PLANT_A = np.array([1.10, 0.55, 0.54, 0.49, 0.39, 0.20, 0., -0.02, -0.12, -0.32, -0.52, -0.64,
                    -0.80, -0.83])
PLANT_TAU = 0.8

# shipped controller constants (mirrored so a config can vary them)
BASE = dict(
    a_lat_max=2.0, plan_margin=0.95, a_budget=0.75, t_lead=1.0, walk_rate=4.0,
    commit_frac=0.7, release_frac=0.3, near_t=3.0,
    kappa_gain_d=None, kappa_gain=None,
    overshoot_bp=[0.02, 0.09, 0.26, 0.44, 0.73], overshoot_gap=[1.5, 2.5, 4.0, 6.0, 8.5],
    max_gap=10.0, min_decel=0.15, rise=10.0, release=3.0,
    react_timer=0.3, deadband=2, dip_gate=True,
)
# measured under-read of far-field model curvature (scc_bias_135.py), as a gain on kappa
BIAS_D = [0., 30., 50., 70., 90., 110., 130., 200.]
BIAS_G = [1.0, 1.06, 1.14, 1.22, 1.42, 1.60, 2.07, 2.07]


def load():
    z = np.load(os.path.join(DIR, 'resampled.npz'), allow_pickle=True)
    d = {k: z[k] for k in z.files}
    with open(os.path.join(DIR, 'model_paths.pkl'), 'rb') as f:
        mp = pickle.load(f)
    with open(os.path.join(DIR, 'episodes.pkl'), 'rb') as f:
        eps = pickle.load(f)
    return d, mp, eps


def prep(d, mp):
    v = d['v']
    s = np.concatenate([[0.], np.cumsum(0.5 * (v[1:] + v[:-1]) * DT)])
    k_real = np.convolve(np.abs(d['curv']), np.ones(5) / 5, mode='same')
    paths = []
    for i in range(len(v)):
        rz = np.abs(np.asarray(mp['rate_z'][i], dtype=float))
        vx = np.asarray(mp['velx'][i], dtype=float)
        px = np.asarray(mp['posx'][i], dtype=float)
        py = np.asarray(mp['posy'][i], dtype=float)
        kap = rz / np.maximum(vx, V_FLOOR)
        dd = np.empty_like(px)
        dd[0] = 0.
        dd[1:] = np.cumsum(np.hypot(np.diff(px), np.diff(py)))
        paths.append((kap, dd))
    return s, k_real, paths


class Servo:
    """ICBM dash servo + decel-overshoot lever, enough of it to be faithful."""

    def __init__(self, cfg, set_mph):
        self.c = cfg
        self.set_mph = set_mph
        self.dash = set_mph
        self.overshoot = 0.
        self.walk_acc = 0.
        self.react = 0.

    def step(self, v_ego, v_target, a_target, limiter, v_ahead_min):
        c = self.c
        want = 0.
        if limiter and a_target < -c['min_decel'] and v_ego > v_target:
            want = min(float(np.interp(-a_target, c['overshoot_bp'], c['overshoot_gap'])), c['max_gap'])
        if want > self.overshoot:
            self.overshoot = min(want, self.overshoot + c['rise'] * DT)
        else:
            rel = c['release'] if limiter else c['rise']
            self.overshoot = max(want, self.overshoot - rel * DT)

        v_cmd = min(v_target, self.set_mph / MPH)
        if self.overshoot > 0:
            v_cmd = min(v_cmd, max(v_ego, v_target) - self.overshoot / MPH)
        cmd_mph = round(v_cmd * MPH)

        # restore gate: hold the dash down while a dip is still ahead
        dip = c['dip_gate'] and v_ahead_min > 0 and v_ahead_min * MPH < min(v_target * MPH, self.set_mph) - c['deadband']
        err = cmd_mph - self.dash
        if abs(err) < c['deadband'] and not (limiter or self.overshoot > 0):
            err = cmd_mph - self.dash if abs(err) >= 1 else 0
        moving = abs(err) >= (c['deadband'] if (limiter or self.overshoot > 0) else 1)
        if err > 0 and dip:
            moving = False
        if not moving:
            self.react = 0.
            self.walk_acc = 0.
            return self.dash
        self.react += DT
        if self.react < c['react_timer']:
            return self.dash
        self.walk_acc += c['walk_rate'] * DT
        while self.walk_acc >= 1.0 and self.dash != cmd_mph:
            self.walk_acc -= 1.0
            self.dash += 1 if cmd_mph > self.dash else -1
        return self.dash


def solve(cfg, kap, dd, v_ego, v_set_ms):
    """The shipped vision solver, with the config's knobs."""
    c = cfg
    k = kap
    if c['kappa_gain'] is not None:
        k = k * np.interp(dd, c['kappa_gain_d'], c['kappa_gain'])
    v_allowed = allowed_speed(k, c['a_lat_max'] * c['plan_margin'])
    t_lead = c['t_lead']
    v_dip = float(np.min(v_allowed))
    if np.isfinite(v_dip):
        t_lead += max(v_ego - max(v_dip, MIN_V), 0.) * MPH / max(c['walk_rate'], 1.)
    d_lead = lead_distance(v_ego, t_lead, c['a_budget'], 0.)
    a_req = required_decel(v_ego, v_allowed, dd, d_lead)
    v_max = backward_pass(v_allowed, dd, c['a_budget'])
    v_now = float(v_max[0])
    v_dip_ahead = min_profile_speed(v_max, dd, float(dd[-1]))
    near = dd <= max(v_ego, MIN_V) * c['near_t']
    v_near = float(np.min(v_allowed[near])) if np.any(near) else float('inf')
    return a_req, v_now, v_dip_ahead, v_near


def run(cfg, d, s, k_real, paths, i0, i1, set_mph, map_v=None):
    """Simulate [i0, i1) of the route. Returns per-step arrays keyed on sim time.

    map_v, when given, is the recorded sccMap target as a function of route distance. The
    map planner reads OSM node curvature at a GPS position, so its target is a property of
    the road, replayable against a different speed profile exactly as the model path is.
    """
    v = float(d['v'][i0])
    pos = float(s[i0])
    servo = Servo(cfg, set_mph)
    # start the dash where the real drive had it, not at the driver setpoint: a curvy road
    # hands each curve a dash the previous one already walked down
    servo.dash = float(np.round(d['set'][i0] * MPH))
    servo.overshoot = max(min((d['v'][i0] - d['set'][i0]) * MPH, cfg['max_gap']), 0.)
    a_state = float(d['a'][i0])
    active = False
    out = dict(t=[], s=[], v=[], dash=[], a=[], lat=[], src=[], vT=[])
    n_max = int((i1 - i0) * 3)
    t = 0.
    s_end = s[i1 - 1]
    for _ in range(n_max):
        if pos >= s_end:
            break
        k = int(np.clip(np.searchsorted(s, pos), 0, len(s) - 1))
        kap, dd = paths[k]
        a_req, v_now, v_dip, v_near = solve(cfg, kap, dd, v, set_mph / MPH)
        if map_v is not None:
            v_map = float(np.interp(pos, s, map_v))
            if v_map < v_now:
                v_now = v_map
            if v_map < v_dip:
                v_dip = v_map
            if v_map < v:
                a_req = max(a_req, (v ** 2 - v_map ** 2) / (2. * max(v * cfg['t_lead'], 5.)))

        commit = a_req >= cfg['commit_frac'] * cfg['a_budget']
        in_curve = np.isfinite(v_near) and v_near < set_mph / MPH
        hold = a_req >= cfg['release_frac'] * cfg['a_budget'] or in_curve
        active = commit or (active and hold)

        if active and v > MIN_V:
            a_need = a_req
            if np.isfinite(v_dip):
                a_need = min(a_need, max(v - v_dip, 0.))
            a_tgt = max(-a_need, -2.0)
            v_lead = v + a_tgt
            v_tgt = max(min(v_now, v_lead, set_mph / MPH), MIN_V)
            if np.isfinite(v_dip):
                v_tgt = min(v_tgt, v_dip)
            limiter = v_tgt < set_mph / MPH - 0.1
        else:
            a_tgt, v_tgt, limiter = 0., set_mph / MPH, False

        v_ahead = min(v_dip, 255.) if np.isfinite(v_dip) else 255.
        dash = servo.step(v, v_tgt, a_tgt, limiter, v_ahead)

        gap = (v - dash / MPH) * MPH
        a_ss = float(np.interp(gap, PLANT_GAP, PLANT_A))
        a_state += (DT / PLANT_TAU) * (a_ss - a_state)

        v = max(v + a_state * DT, 1.0)
        pos += v * DT
        t += DT
        out['t'].append(t); out['s'].append(pos); out['v'].append(v)
        out['dash'].append(dash); out['a'].append(a_state)
        out['lat'].append(v * v * float(np.interp(pos, s, k_real)))
        out['src'].append(limiter); out['vT'].append(v_tgt)
    return {k: np.asarray(val) for k, val in out.items()}


def score(cfg, d, s, k_real, paths, eps, map_v=None, run_in_s=14.0):
    rows = []
    for e in eps:
        i, hi = e['i'], min(e['hi'] + 40, len(s) - 1)
        lo = max(5, i - int(run_in_s / DT))
        if not d['eng'][lo:i].all():
            lo = e['lo']
        # the driver's own setpoint: the dash ceiling the servo restores toward
        set_mph = float(np.round(np.max(d['set'][max(0, lo - 200):hi]) * MPH))
        r = run(cfg, d, s, k_real, paths, lo, hi, set_mph, map_v)
        if len(r['s']) < 5:
            continue
        s_apex = s[i]
        j = int(np.argmin(np.abs(r['s'] - s_apex)))
        v_sim = r['v'][j]
        lat_sim = v_sim ** 2 * k_real[i]
        t_real = (hi - lo) * DT
        rows.append(dict(t=e['t'], lat_real=e['lat_apex'], lat_sim=lat_sim, hi=hi,
                         v_real=e['v_apex'], v_sim=v_sim, v_allowed=e['v_allowed'],
                         dt=r['t'][-1] - t_real, set_mph=set_mph, r=r, i=i, lo=lo))
    return rows


def summarize(name, rows):
    lat = np.array([r['lat_sim'] for r in rows])
    dt = np.array([r['dt'] for r in rows])
    return (f"{name:26} n={len(rows):3}  apex latA: med {np.median(lat):.2f} p90 {np.percentile(lat,90):.2f} "
            f"max {lat.max():.2f}  >2.0: {(lat>2.0).sum():2}  >2.2: {(lat>2.2).sum():2}  "
            f"time {dt.sum():+.1f}s over {len(rows)} run-ins")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--detail', type=float, default=None, help='dump the sim for the apex at this t')
    ap.add_argument('--validate', action='store_true', help='score the sim against the recorded drive')
    args = ap.parse_args()

    d, mp, eps = load()
    s, k_real, paths = prep(d, mp)
    eps = [e for e in eps if e['lo'] > 5]
    # sccMap's recorded target, as a road property indexed on distance
    map_v = np.where(np.isin(d['mapState'], ('turning',)), d['mapVT'], 1e3)
    print(f"{len(eps)} episodes; map constraint active on "
          f"{(map_v < 100).sum() / len(map_v) * 100:.1f}% of frames\n")

    if args.validate:
        print("=== sim vs recorded, baseline config ===")
        print(f"  {'t':>7} {'latReal':>7} {'latSim':>7} {'vReal':>6} {'vSim':>6} "
              f"{'dvApex':>6} {'dt':>6}")
        rows = score(dict(BASE), d, s, k_real, paths, eps, map_v)
        e_lat, e_v = [], []
        for r in rows:
            e_lat.append(r['lat_sim'] - r['lat_real'])
            e_v.append((r['v_sim'] - r['v_real']) * MPH)
            print(f"  {r['t']:7.1f} {r['lat_real']:7.2f} {r['lat_sim']:7.2f} "
                  f"{r['v_real']*MPH:6.1f} {r['v_sim']*MPH:6.1f} "
                  f"{(r['v_sim']-r['v_real'])*MPH:+6.1f} {r['dt']:+6.1f}")
        e_lat, e_v = np.array(e_lat), np.array(e_v)
        print(f"\n  apex speed error: median {np.median(e_v):+.1f} mph  "
              f"MAE {np.abs(e_v).mean():.1f} mph  rms {np.sqrt((e_v**2).mean()):.1f}")
        print(f"  apex latA error : median {np.median(e_lat):+.2f}  "
              f"MAE {np.abs(e_lat).mean():.2f}")
        return

    print("recorded baseline:")
    lat = np.array([e['lat_apex'] for e in eps])
    print(f"{'AS FLOWN':26} n={len(eps):3}  apex latA: med {np.median(lat):.2f} "
          f"p90 {np.percentile(lat,90):.2f} max {lat.max():.2f}  >2.0: {(lat>2.0).sum():2}  "
          f">2.2: {(lat>2.2).sum():2}\n")

    cfgs = {}
    cfgs['baseline (shipped)'] = dict(BASE)
    c = dict(BASE); c['a_budget'] = 0.60; cfgs['budget 0.60'] = c
    c = dict(BASE); c['a_budget'] = 0.50; cfgs['budget 0.50'] = c
    c = dict(BASE); c['kappa_gain_d'] = BIAS_D; c['kappa_gain'] = BIAS_G; cfgs['kappa bias gain'] = c
    c = dict(BASE); c['kappa_gain_d'] = BIAS_D; c['kappa_gain'] = [min(g, 1.5) for g in BIAS_G]
    cfgs['kappa gain (capped 1.5)'] = c
    c = dict(BASE); c['a_budget'] = 0.60; c['kappa_gain_d'] = BIAS_D
    c['kappa_gain'] = [min(g, 1.5) for g in BIAS_G]; cfgs['budget 0.60 + gain 1.5'] = c
    c = dict(BASE); c['commit_frac'] = 0.5; cfgs['commit_frac 0.5'] = c
    c = dict(BASE); c['plan_margin'] = 0.85; cfgs['plan_margin 0.85'] = c
    c = dict(BASE); c['overshoot_gap'] = [1.5, 2.0, 3.0, 4.5, 6.0]; cfgs['deeper gap earlier'] = c

    results = {}
    for name, cfg in cfgs.items():
        rows = score(cfg, d, s, k_real, paths, eps, map_v)
        results[name] = rows
        print(summarize(name, rows))

    print("\n=== per-apex, worst first (sim latA) ===")
    base = {r['t']: r for r in results['baseline (shipped)']}
    keys = list(cfgs)
    hdr = f"{'t':>7} {'real':>5} " + ''.join(f"{n[:11]:>12}" for n in keys)
    print(hdr)
    for e in sorted(eps, key=lambda e: -e['lat_apex']):
        line = f"{e['t']:7.1f} {e['lat_apex']:5.2f} "
        for n in keys:
            m = [r for r in results[n] if r['t'] == e['t']]
            line += f"{m[0]['lat_sim']:12.2f}" if m else f"{'-':>12}"
        print(line)

    if args.detail is not None:
        for n in keys:
            m = [r for r in results[n] if abs(r['t'] - args.detail) < 0.5]
            if not m:
                continue
            r = m[0]['r']
            print(f"\n--- {n}  (set {m[0]['set_mph']:.0f} mph) ---")
            print(f"  {'t':>6} {'v':>6} {'dash':>5} {'gap':>5} {'a':>6} {'lat':>5} {'lim':>4}")
            for k in range(0, len(r['t']), 8):
                print(f"  {r['t'][k]:6.2f} {r['v'][k]*MPH:6.1f} {r['dash'][k]:5.0f} "
                      f"{r['v'][k]*MPH-r['dash'][k]:5.1f} {r['a'][k]:6.2f} {r['lat'][k]:5.2f} "
                      f"{int(r['src'][k]):4}")


if __name__ == '__main__':
    main()
