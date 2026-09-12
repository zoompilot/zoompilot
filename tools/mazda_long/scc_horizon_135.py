"""Route 135: how early is a curve actually visible in the model path?

The planner brakes when the backward pass over the model path drops below the setpoint.
If the model under-reads far-field curvature, the commit is structurally late no matter
how the gate is tuned. For each apex, walk back through the run-in and ask what the model
said the allowed speed was at the distance the apex actually sat at.
"""
import os, pickle
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694
DT = 0.05
A_LAT = 2.0
V_FLOOR = 0.5
KAPPA_MIN = 1e-5


def load():
    z = np.load(os.path.join(DIR, 'resampled.npz'), allow_pickle=True)
    d = {k: z[k] for k in z.files}
    with open(os.path.join(DIR, 'model_paths.pkl'), 'rb') as f:
        mp = pickle.load(f)
    with open(os.path.join(DIR, 'episodes.pkl'), 'rb') as f:
        eps = pickle.load(f)
    return d, mp, eps


def path_kappa(mp, i):
    rz = np.abs(np.asarray(mp['rate_z'][i], dtype=float))
    vx = np.asarray(mp['velx'][i], dtype=float)
    px = np.asarray(mp['posx'][i], dtype=float)
    py = np.asarray(mp['posy'][i], dtype=float)
    kap = rz / np.maximum(vx, V_FLOOR)
    dist = np.empty_like(px)
    dist[0] = 0.
    dist[1:] = np.cumsum(np.hypot(np.diff(px), np.diff(py)))
    return kap, dist


def main():
    d, mp, eps = load()
    v = d['v']
    curv = np.abs(d['curv'])
    k_meas = np.convolve(curv, np.ones(5) / 5, mode='same')

    print("=== curvature visibility: predicted vs realized apex kappa ===")
    print("For each apex, at each time before it, the distance the apex sat at and the")
    print("model's own kappa near that distance. v_allow = sqrt(2.0/kappa) in mph.\n")

    tt = np.arange(-8.0, 0.01, 0.5)
    tab = {t: [] for t in tt}
    for e in eps:
        i = e['i']
        k_apex = k_meas[i]
        if k_apex < 1e-4:
            continue
        v_allow_true = np.sqrt(A_LAT / k_apex) * MPH
        for t_rel in tt:
            j = i + int(round(t_rel / DT))
            if j < 0 or j >= len(v):
                continue
            # distance travelled from j to apex along the ground
            dist_to_apex = np.trapezoid(v[j:i + 1], dx=DT) if i > j else 0.
            kap, dist = path_kappa(mp, j)
            if dist[-1] < dist_to_apex:
                tab[t_rel].append((v_allow_true, np.nan, dist_to_apex, dist[-1]))
                continue
            # model kappa in a +-15 m window around where the apex is
            w = (dist >= dist_to_apex - 15) & (dist <= dist_to_apex + 15)
            k_pred = float(kap[w].max()) if w.any() else np.nan
            v_allow_pred = np.sqrt(A_LAT / max(k_pred, KAPPA_MIN)) * MPH
            tab[t_rel].append((v_allow_true, v_allow_pred, dist_to_apex, dist[-1]))

    print(f"  {'t-apex':>7} {'n':>4} {'d_apex':>7} {'horiz':>7} {'inHoriz':>8} "
          f"{'vAllwTrue':>10} {'vAllwPred':>10} {'ratio':>6} {'medErr':>7}")
    for t_rel in tt:
        rows = tab[t_rel]
        if not rows:
            continue
        arr = np.array([[a, b, c, dd] for a, b, c, dd in rows], dtype=float)
        inh = np.isfinite(arr[:, 1])
        if inh.sum() == 0:
            continue
        true_v, pred_v = arr[inh, 0], arr[inh, 1]
        print(f"  {t_rel:7.1f} {len(rows):4} {np.median(arr[:,2]):7.0f} {np.median(arr[:,3]):7.0f} "
              f"{inh.sum():8} {np.median(true_v):10.1f} {np.median(pred_v):10.1f} "
              f"{np.median(pred_v/true_v):6.2f} {np.median(pred_v-true_v):+7.1f}")

    # how far ahead the profile first drops below the setpoint
    print("\n=== when the plan first binds, per episode ===")
    print(f"  {'t_apex':>7} {'latApex':>7} {'setPre':>6} {'firstBind':>9} {'firstSrc':>8} "
          f"{'needTime':>8} {'haveTime':>8} {'deficit':>7}")
    a_real = 0.62  # measured steady-state MRCC decel
    for e in sorted(eps, key=lambda x: -x['lat_apex']):
        i, lo = e['i'], e['lo']
        setp = e['set_peak_pre']
        # first frame in run-in where vAhead dropped below the dash setpoint
        va = d['vAhead'][lo:i + 1]
        below = va < setp
        first_bind = (np.argmax(below) - (i - lo)) * DT if below.any() else np.nan
        src = np.isin(d['src'][lo:i + 1], ('sccVision', 'sccMap'))
        first_src = (np.argmax(src) - (i - lo)) * DT if src.any() else np.nan
        dv = max(e['v_peak_pre'] - e['v_allowed'], 0.)
        need = dv / a_real
        have = -first_bind if np.isfinite(first_bind) else np.nan
        print(f"  {e['t']:7.1f} {e['lat_apex']:7.2f} {setp*MPH:6.0f} {first_bind:9.2f} "
              f"{first_src:8.2f} {need:8.2f} {have:8.2f} {(need-have) if np.isfinite(have) else float('nan'):+7.2f}")


if __name__ == '__main__':
    main()
