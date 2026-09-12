"""Route 135 curve-episode analysis.

Stock MRCC path (opLong=False): every deceleration goes through the ICBM dash servo plus
the decel-overshoot lever. For each curve apex the drive passed through engaged, measure
whether we arrived at the allowed speed and, where we did not, which link in the chain
gave out: the commit gate, the dash walk, or the ECU's own response.
"""
import os
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694
A_LAT_MAX = 2.0
DT = 0.05  # model rate


def load():
    z = np.load(os.path.join(DIR, 'resampled.npz'), allow_pickle=True)
    return {k: z[k] for k in z.files}


def find_apexes(d, min_lat=1.2, min_gap_s=2.0):
    """Curve apexes = local maxima of |curvature| while engaged and moving."""
    v, curv, eng = d['v'], np.abs(d['curv']), d['eng']
    lat = v ** 2 * curv
    ok = eng & (v > 8.0)  # ~18 mph; below MIN_V SCC does not act anyway
    # smooth curvature a little; sensor noise makes spurious peaks
    k = np.convolve(curv, np.ones(5) / 5, mode='same')
    cand = []
    n = len(k)
    for i in range(2, n - 2):
        if not ok[i]:
            continue
        if lat[i] < min_lat:
            continue
        w = slice(max(0, i - 10), min(n, i + 11))
        if k[i] >= k[w].max():
            cand.append(i)
    # collapse to one apex per curve
    apex = []
    gap = int(min_gap_s / DT)
    for i in cand:
        if apex and i - apex[-1] < gap:
            if lat[i] > lat[apex[-1]]:
                apex[-1] = i
            continue
        apex.append(i)
    return np.array(apex)


def episode_rows(d, apexes, pre_s=8.0, post_s=3.0):
    v, curv, eng = d['v'], np.abs(d['curv']), d['eng']
    lat = v ** 2 * curv
    setv = d['set']
    rows = []
    pre = int(pre_s / DT)
    post = int(post_s / DT)
    for i in apexes:
        lo, hi = max(0, i - pre), min(len(v), i + post)
        sl = slice(lo, hi)
        if not eng[sl].all():
            continue
        kap = curv[i]
        v_allowed = np.sqrt(A_LAT_MAX / max(kap, 1e-5))
        # how much the plan wanted, and what the servo/ECU produced, in the run-in
        src_win = d['src'][sl]
        scc_win = d['sccState'][sl]
        active = np.isin(src_win, ('sccVision', 'sccMap'))
        # commit index: first frame in window where the SCC owned the source
        commit_rel = int(np.argmax(active)) if active.any() else None
        rows.append(dict(
            i=i, t=d['t'][i],
            v_apex=v[i], v_allowed=v_allowed, lat_apex=lat[i],
            kappa=kap,
            v_peak_pre=v[lo:i].max() if i > lo else v[i],
            set_apex=setv[i], set_peak_pre=setv[lo:i].max() if i > lo else setv[i],
            scc_owned=bool(active.any()),
            scc_frames=int(active.sum()),
            commit_lead_s=(i - lo - commit_rel) * DT if commit_rel is not None else np.nan,
            scc_state_apex=str(d['sccState'][i]),
            src_apex=str(d['src'][i]),
            vT_min=d['vT'][sl].min(), aT_min=d['aT'][sl].min(),
            v_ahead_min=d['vAhead'][sl].min(),
            lead=bool(d['lead'][sl].any()),
            lead_d=float(np.nanmin(np.where(d['lead'][sl], d['leadD'][sl], np.nan))) if d['lead'][sl].any() else np.nan,
            a_mean_run_in=float(np.mean(np.gradient(v[lo:i + 1], DT))) if i > lo else 0.,
            a_min_run_in=float(np.min(np.gradient(v[lo:i + 1], DT))) if i - lo > 3 else 0.,
            gap_max=float(np.max(v[sl] - setv[sl])),
            lo=lo, hi=hi,
        ))
    return rows


def main():
    d = load()
    apexes = find_apexes(d)
    rows = episode_rows(d, apexes)
    print(f"{len(apexes)} apexes, {len(rows)} fully-engaged episodes\n")

    over = [r for r in rows if r['lat_apex'] > 2.0]
    fair = [r for r in rows if not r['lead']]
    print(f"apex lat accel: median {np.median([r['lat_apex'] for r in rows]):.2f}  "
          f"max {max(r['lat_apex'] for r in rows):.2f}")
    print(f"over 2.0: {len(over)}/{len(rows)}   over 2.2: {sum(r['lat_apex']>2.2 for r in rows)}   "
          f"over 2.5: {sum(r['lat_apex']>2.5 for r in rows)}")
    print(f"SCC owned the source somewhere in run-in: {sum(r['scc_owned'] for r in rows)}/{len(rows)}")
    print(f"episodes with a lead: {sum(r['lead'] for r in rows)}\n")

    hdr = (f"{'t':>7} {'vApex':>6} {'vAllw':>6} {'err':>5} {'latA':>5} {'set@ap':>6} "
           f"{'setPre':>6} {'vPre':>5} {'gapMx':>5} {'scc':>4} {'lead_s':>6} {'aMin':>5} "
           f"{'aMean':>5} {'state':>9} {'src':>10} {'lead':>5}")
    print(hdr)
    print('-' * len(hdr))
    for r in sorted(rows, key=lambda r: -r['lat_apex']):
        err = (r['v_apex'] - r['v_allowed']) * MPH
        print(f"{r['t']:7.1f} {r['v_apex']*MPH:6.1f} {r['v_allowed']*MPH:6.1f} {err:+5.1f} "
              f"{r['lat_apex']:5.2f} {r['set_apex']*MPH:6.1f} {r['set_peak_pre']*MPH:6.1f} "
              f"{r['v_peak_pre']*MPH:5.1f} {r['gap_max']*MPH:5.1f} {r['scc_frames']:4} "
              f"{r['commit_lead_s']:6.2f} {r['a_min_run_in']:5.2f} {r['a_mean_run_in']:5.2f} "
              f"{r['scc_state_apex'][:9]:>9} {r['src_apex'][:10]:>10} "
              f"{('%.0f' % r['lead_d']) if r['lead'] else '-':>5}")

    np.save(os.path.join(DIR, 'apexes.npy'), apexes)
    import pickle
    with open(os.path.join(DIR, 'episodes.pkl'), 'wb') as f:
        pickle.dump(rows, f)
    print(f"\nsaved {len(rows)} episodes")


if __name__ == '__main__':
    main()
