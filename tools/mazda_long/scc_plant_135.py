"""Route 135: fit a plant model of the Mazda MRCC, gap -> longitudinal accel.

Needed so candidate SCC changes can be simulated closed-loop rather than argued about.
Model: a_ss = f(gap) through a first-order lag of time constant tau.
Fits f as a piecewise-linear table and tau by sweep, scored on held-out episodes.
"""
import os, json
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694
DT = 0.05
GAP_BP = np.array([-14., -10., -7., -5., -3., -1.5, 0., 1.5, 3., 4., 5., 6., 7., 8., 10., 14.])


def load():
    z = np.load(os.path.join(DIR, 'resampled.npz'), allow_pickle=True)
    return {k: z[k] for k in z.files}


def lag(u, tau):
    """First-order lag, zero-order hold."""
    alpha = DT / max(tau, 1e-3)
    y = np.empty_like(u)
    acc = u[0]
    for i in range(len(u)):
        acc += alpha * (u[i] - acc)
        y[i] = acc
    return y


def main():
    d = load()
    v, setv, eng = d['v'], d['set'], d['eng']
    gap = (v - setv) * MPH
    a_meas = np.convolve(np.gradient(v, DT), np.ones(9) / 9, mode='same')

    # a lead only matters when it is close enough for the ECU to be following it
    following = d['lead'] & (d['leadD'] < 60)
    ok = eng & ~d['gas'] & ~d['brake'] & ~following & (v > 8)
    # contiguous usable stretches
    runs = []
    i = 0
    while i < len(ok):
        if ok[i]:
            j = i
            while j < len(ok) and ok[j]:
                j += 1
            if (j - i) * DT >= 6:
                runs.append((i, j))
            i = j
        else:
            i += 1
    print(f"{len(runs)} usable stretches, {sum(j-i for i,j in runs)*DT:.0f}s total")

    # alternate stretches -> train / test
    train = runs[0::2]
    test = runs[1::2]

    def fit_table(rs, tau):
        """Least squares for the table values given tau (linear in the table)."""
        rows, ys = [], []
        for i, j in rs:
            g = gap[i:j]
            # basis: lagged response of each hat function
            B = np.zeros((j - i, len(GAP_BP)))
            for k in range(len(GAP_BP)):
                e = np.zeros(len(GAP_BP)); e[k] = 1.
                B[:, k] = lag(np.interp(g, GAP_BP, e), tau)
            rows.append(B); ys.append(a_meas[i:j])
        A = np.vstack(rows); y = np.concatenate(ys)
        # constrain: f(0) = 0 and monotone non-increasing. Solve in increments
        # (f = C @ delta, delta <= 0) by projected gradient; no scipy on this box.
        n = len(GAP_BP)
        C = np.tril(np.ones((n, n)))
        AC = A @ C
        H = AC.T @ AC
        g0 = AC.T @ y
        step = 1.0 / (np.linalg.eigvalsh(H).max() + 1e-9)
        zero = int(np.argmin(np.abs(GAP_BP)))
        x = np.zeros(n)
        for _ in range(30000):
            x = x - step * (H @ x - g0)
            x[1:] = np.minimum(x[1:], 0.)   # monotone non-increasing in gap
            f = C @ x
            x[0] -= f[zero]                 # pin f(gap=0) = 0
        return C @ x

    def score(rs, tbl, tau):
        errs = []
        for i, j in rs:
            pred = lag(np.interp(gap[i:j], GAP_BP, tbl), tau)
            errs.append(pred - a_meas[i:j])
        e = np.concatenate(errs)
        return float(np.sqrt((e ** 2).mean()))

    print(f"\n{'tau':>6} {'rmse train':>11} {'rmse test':>10}")
    best = None
    for tau in (0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 1.8, 2.2, 2.8):
        tbl = fit_table(train, tau)
        rtr, rte = score(train, tbl, tau), score(test, tbl, tau)
        print(f"{tau:6.1f} {rtr:11.3f} {rte:10.3f}")
        if best is None or rte < best[0]:
            best = (rte, tau, tbl)

    rte, tau, _ = best
    tbl = fit_table(runs, tau)  # refit on everything at the chosen tau
    print(f"\nbest tau = {tau:.1f}s  (test rmse {rte:.3f} m/s2)")
    print(f"\nfitted steady-state response:")
    print(f"  {'gap mph':>8} {'a m/s2':>8}")
    for g, a in zip(GAP_BP, tbl):
        print(f"  {g:8.1f} {a:8.3f}")

    # marginal value
  

    # effective budget over a realistic 8 s maneuver from a standing start of the gap
    print(f"\n=== effective budget over a maneuver (dash walks at 4 mph/s, then holds) ===")
    for hold_gap in (5, 6, 7, 8, 10):
        n = int(12 / DT)
        g = np.minimum(np.arange(n) * DT * 4.0, hold_gap)
        a_sim = lag(np.interp(g, GAP_BP, tbl), tau)
        for horizon in (4, 6, 8, 10):
            k = int(horizon / DT)
            print(f"  gap->{hold_gap:2} mph, first {horizon:2}s: mean a = {a_sim[:k].mean():.3f}", end='')
        print()

    with open(os.path.join(DIR, 'plant.json'), 'w') as f:
        json.dump({'gap_bp': GAP_BP.tolist(), 'a_ss': tbl.tolist(), 'tau': tau}, f, indent=2)
    print(f"\nwrote {DIR}/plant.json")


if __name__ == '__main__':
    main()
