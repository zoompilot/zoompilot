"""Route 135: is there a better far-field curvature estimator than rate_z / v?

Compares three estimates of path curvature against the curvature the car actually
realized when it got there:
  A. rate_z / velocity.x          (shipped)
  B. geometric, from position.x/y (finite-difference curvature of the polyline)
  C. geometric on a smoothed polyline
"""
import os, pickle
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694
DT = 0.05
A_LAT = 2.0
V_FLOOR = 0.5


def load():
    z = np.load(os.path.join(DIR, 'resampled.npz'), allow_pickle=True)
    d = {k: z[k] for k in z.files}
    with open(os.path.join(DIR, 'model_paths.pkl'), 'rb') as f:
        mp = pickle.load(f)
    with open(os.path.join(DIR, 'episodes.pkl'), 'rb') as f:
        eps = pickle.load(f)
    return d, mp, eps


def est_rate(mp, i):
    rz = np.abs(np.asarray(mp['rate_z'][i], dtype=float))
    vx = np.asarray(mp['velx'][i], dtype=float)
    return rz / np.maximum(vx, V_FLOOR)


def est_geom(mp, i, smooth=0):
    px = np.asarray(mp['posx'][i], dtype=float)
    py = np.asarray(mp['posy'][i], dtype=float)
    if smooth:
        k = np.ones(smooth) / smooth
        px = np.convolve(px, k, mode='same')
        py = np.convolve(py, k, mode='same')
        px[:smooth], px[-smooth:] = np.asarray(mp['posx'][i], dtype=float)[:smooth], np.asarray(mp['posx'][i], dtype=float)[-smooth:]
        py[:smooth], py[-smooth:] = np.asarray(mp['posy'][i], dtype=float)[:smooth], np.asarray(mp['posy'][i], dtype=float)[-smooth:]
    dx = np.gradient(px)
    dy = np.gradient(py)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = np.power(dx * dx + dy * dy, 1.5)
    return np.abs(dx * ddy - dy * ddx) / np.maximum(denom, 1e-6)


def dist_of(mp, i):
    px = np.asarray(mp['posx'][i], dtype=float)
    py = np.asarray(mp['posy'][i], dtype=float)
    dd = np.empty_like(px)
    dd[0] = 0.
    dd[1:] = np.cumsum(np.hypot(np.diff(px), np.diff(py)))
    return dd


def main():
    d, mp, eps = load()
    v = d['v']
    k_meas = np.convolve(np.abs(d['curv']), np.ones(5) / 5, mode='same')

    ests = {'rate_z/v': est_rate,
            'geom': lambda m, i: est_geom(m, i, 0),
            'geom_s5': lambda m, i: est_geom(m, i, 5)}

    tt = np.arange(-8.0, 0.01, 1.0)
    res = {name: {t: [] for t in tt} for name in ests}
    for e in eps:
        i = e['i']
        k_apex = k_meas[i]
        if k_apex < 1e-4:
            continue
        for t_rel in tt:
            j = i + int(round(t_rel / DT))
            if j < 0 or j >= len(v):
                continue
            d_apex = np.trapezoid(v[j:i + 1], dx=DT) if i > j else 0.
            dd = dist_of(mp, j)
            if dd[-1] < d_apex:
                continue
            w = (dd >= d_apex - 15) & (dd <= d_apex + 15)
            if not w.any():
                continue
            for name, fn in ests.items():
                kp = fn(mp, j)
                res[name][t_rel].append((k_apex, float(kp[w].max())))

    print("=== apex kappa: model estimate / realized, by time-to-apex ===")
    print("  (ratio < 1 = model under-reads the corner; that is late braking)\n")
    hdr = f"  {'t-apex':>7}" + ''.join(f"{n:>22}" for n in ests)
    print(hdr)
    print(f"  {'':>7}" + ''.join(f"{'ratio':>10}{'v_err mph':>12}" for _ in ests))
    for t_rel in tt:
        line = f"  {t_rel:7.1f}"
        for name in ests:
            rows = res[name][t_rel]
            if not rows:
                line += f"{'-':>10}{'-':>12}"
                continue
            a = np.array(rows)
            ratio = np.median(a[:, 1] / a[:, 0])
            v_true = np.sqrt(A_LAT / a[:, 0]) * MPH
            v_pred = np.sqrt(A_LAT / np.maximum(a[:, 1], 1e-5)) * MPH
            line += f"{ratio:10.2f}{np.median(v_pred - v_true):12.1f}"
        print(line)

    # false positives: what does each estimator claim on genuinely straight road?
    print("\n=== straight-road behaviour (no apex within +-6 s, engaged, v>15 m/s) ===")
    apex_i = np.array([e['i'] for e in eps])
    near_apex = np.zeros(len(v), dtype=bool)
    for i in apex_i:
        near_apex[max(0, i - 160):min(len(v), i + 160)] = True
    straight = d['eng'] & ~near_apex & (v > 15)
    idx = np.where(straight)[0]
    idx = idx[::5][:600]
    print(f"  {len(idx)} sampled frames")
    for name, fn in ests.items():
        mins = []
        for j in idx:
            kp = fn(mp, j)
            dd = dist_of(mp, j)
            far = dd > 20
            if not far.any():
                continue
            mins.append(np.sqrt(A_LAT / max(float(kp[far].max()), 1e-5)) * MPH)
        mins = np.array(mins)
        print(f"  {name:10}  min v_allow on straight road: p5 {np.percentile(mins,5):5.1f} "
              f"p25 {np.percentile(mins,25):5.1f} median {np.median(mins):6.1f} mph   "
              f"frames claiming < 50 mph: {(mins<50).sum()}/{len(mins)}")


if __name__ == '__main__':
    main()
