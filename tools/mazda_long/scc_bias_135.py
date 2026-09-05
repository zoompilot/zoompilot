"""Route 135: is the far-field curvature under-read a function of distance, of path
index (model time), or of speed? The answer decides what the correction is indexed on."""
import os, pickle
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694
DT = 0.05
V_FLOOR = 0.5
# openpilot model time index grid
T_IDXS = np.array([0., 0.00976562, 0.0390625, 0.08789062, 0.15625, 0.24414062, 0.3515625,
                   0.47851562, 0.625, 0.79101562, 0.9765625, 1.18164062, 1.40625, 1.65039062,
                   1.9140625, 2.19726562, 2.5, 2.82226562, 3.1640625, 3.52539062, 3.90625,
                   4.30664062, 4.7265625, 5.16601562, 5.625, 6.10351562, 6.6015625, 7.11914062,
                   7.65625, 8.21289062, 8.7890625, 9.38476562, 10.])


def load():
    z = np.load(os.path.join(DIR, 'resampled.npz'), allow_pickle=True)
    d = {k: z[k] for k in z.files}
    with open(os.path.join(DIR, 'model_paths.pkl'), 'rb') as f:
        mp = pickle.load(f)
    with open(os.path.join(DIR, 'episodes.pkl'), 'rb') as f:
        eps = pickle.load(f)
    return d, mp, eps


def kappa_dist(mp, i):
    rz = np.abs(np.asarray(mp['rate_z'][i], dtype=float))
    vx = np.asarray(mp['velx'][i], dtype=float)
    px = np.asarray(mp['posx'][i], dtype=float)
    py = np.asarray(mp['posy'][i], dtype=float)
    kap = rz / np.maximum(vx, V_FLOOR)
    dd = np.empty_like(px)
    dd[0] = 0.
    dd[1:] = np.cumsum(np.hypot(np.diff(px), np.diff(py)))
    return kap, dd


def main():
    d, mp, eps = load()
    v = d['v']
    k_meas = np.convolve(np.abs(d['curv']), np.ones(5) / 5, mode='same')

    # collect (distance, time-ahead, speed, ratio) samples
    samples = []
    for e in eps:
        i = e['i']
        k_apex = k_meas[i]
        if k_apex < 2e-4:
            continue
        for back in range(1, 201):  # up to 10 s back at 20 Hz
            j = i - back
            if j < 0 or not d['eng'][j]:
                continue
            d_apex = np.trapezoid(v[j:i + 1], dx=DT)
            kap, dd = kappa_dist(mp, j)
            if dd[-1] < d_apex or d_apex < 3:
                continue
            w = (dd >= d_apex - 12) & (dd <= d_apex + 12)
            if not w.any():
                continue
            k_pred = float(kap[w].max())
            samples.append((d_apex, back * DT, v[j], k_pred / k_apex, k_apex))
    s = np.array(samples)
    print(f"{len(s)} samples from {len(eps)} apexes\n")
    dist, tahead, spd, ratio, kap_true = s.T

    print("=== ratio (model kappa / realized) by DISTANCE to the apex ===")
    print(f"  {'dist m':>10} {'n':>5} {'ratio':>7} {'p25':>6} {'p75':>6} {'gain 1/r':>9}")
    edges = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 115, 130, 150, 200]
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (dist >= lo) & (dist < hi)
        if m.sum() < 15:
            continue
        r = np.median(ratio[m])
        print(f"  {lo:4}-{hi:<5} {m.sum():5} {r:7.2f} {np.percentile(ratio[m],25):6.2f} "
              f"{np.percentile(ratio[m],75):6.2f} {1/max(r,1e-3):9.2f}")

    print("\n=== same, split by speed (does it track distance or time?) ===")
    for vlo, vhi, lbl in ((8, 14, '18-31 mph'), (14, 19, '31-42 mph'), (19, 30, '42+ mph')):
        m0 = (spd >= vlo) & (spd < vhi)
        if m0.sum() < 40:
            continue
        print(f"\n  {lbl}  (n={m0.sum()})")
        print(f"    {'dist m':>10} {'n':>5} {'ratio':>7}   |   {'t ahead':>8} {'n':>5} {'ratio':>7}")
        for k in range(7):
            dlo, dhi = k * 20, (k + 1) * 20
            md = m0 & (dist >= dlo) & (dist < dhi)
            tlo, thi = k * 1.0, (k + 1) * 1.0
            mt = m0 & (tahead >= tlo) & (tahead < thi)
            dcell = f"{np.median(ratio[md]):7.2f}" if md.sum() >= 10 else f"{'-':>7}"
            tcell = f"{np.median(ratio[mt]):7.2f}" if mt.sum() >= 10 else f"{'-':>7}"
            print(f"    {dlo:4}-{dhi:<5} {md.sum():5} {dcell}   |   {tlo:4.0f}-{thi:<3.0f} {mt.sum():5} {tcell}")

    print("\n=== by model path index (fraction of the 10 s horizon) ===")
    # where along its own path does the apex sit?
    frac = []
    for e in eps:
        i = e['i']
        if k_meas[i] < 2e-4:
            continue
        for back in range(1, 201):
            j = i - back
            if j < 0 or not d['eng'][j]:
                continue
            d_apex = np.trapezoid(v[j:i + 1], dx=DT)
            kap, dd = kappa_dist(mp, j)
            if dd[-1] < d_apex or d_apex < 3:
                continue
            w = (dd >= d_apex - 12) & (dd <= d_apex + 12)
            if not w.any():
                continue
            idx = int(np.argmin(np.abs(dd - d_apex)))
            frac.append((idx / (len(dd) - 1), float(kap[w].max()) / k_meas[i]))
    f = np.array(frac)
    print(f"  {'idx frac':>10} {'n':>5} {'ratio':>7}")
    for lo, hi in zip(np.arange(0, 1.0, 0.1), np.arange(0.1, 1.01, 0.1)):
        m = (f[:, 0] >= lo) & (f[:, 0] < hi)
        if m.sum() < 15:
            continue
        print(f"  {lo:4.1f}-{hi:<5.1f} {m.sum():5} {np.median(f[m,1]):7.2f}")

    print("\n=== also by curve severity (does the bias depend on how tight it is?) ===")
    print(f"  {'v_allow mph':>12} {'n':>5} {'ratio@60-100m':>14}")
    band = (dist >= 60) & (dist < 100)
    va = np.sqrt(2.0 / np.maximum(kap_true, 1e-5)) * MPH
    for lo, hi in ((0, 25), (25, 32), (32, 40), (40, 55), (55, 200)):
        m = band & (va >= lo) & (va < hi)
        if m.sum() < 10:
            continue
        print(f"  {lo:4}-{hi:<7} {m.sum():5} {np.median(ratio[m]):14.2f}")


if __name__ == '__main__':
    main()
