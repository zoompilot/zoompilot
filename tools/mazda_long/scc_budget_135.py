"""Route 135: what deceleration can the MRCC actually be made to deliver?

limits.py plans against _STOCK_A_BUDGET['mazda'] = 0.75 m/s2 and the overshoot inverse map
claims 8.5 mph of gap buys 0.73. Measure both against the drive: the sustained ceiling, the
transient peak, and the gap at which the response stops improving.
"""
import os, pickle
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694
DT = 0.05


def load():
    z = np.load(os.path.join(DIR, 'resampled.npz'), allow_pickle=True)
    return {k: z[k] for k in z.files}


def main():
    d = load()
    v, setv, eng = d['v'], d['set'], d['eng']
    gap = (v - setv) * MPH
    # accel from vEgo; smooth over 0.55 s
    a = np.convolve(np.gradient(v, DT), np.ones(11) / 11, mode='same')

    clean = eng & ~d['gas'] & ~d['brake'] & (v > 8)
    no_lead = clean & ~d['lead']
    print(f"engaged clean frames {clean.sum()}, of which no-lead {no_lead.sum()}")

    # sustained ceiling: longest windows of continuous deceleration
    print("\n=== sustained deceleration episodes (no lead, no pedals, >=2 s) ===")
    braking = no_lead & (a < -0.2)
    runs = []
    i = 0
    while i < len(braking):
        if braking[i]:
            j = i
            while j < len(braking) and braking[j]:
                j += 1
            if (j - i) * DT >= 2.0:
                runs.append((i, j))
            i = j
        else:
            i += 1
    print(f"  {len(runs)} runs")
    print(f"  {'t':>7} {'dur':>5} {'gapMed':>7} {'gapMax':>7} {'aMean':>6} {'aMin':>6} {'a p10':>6} {'dv mph':>7}")
    peaks, means = [], []
    for i, j in runs:
        sl = slice(i, j)
        peaks.append(a[sl].min()); means.append(a[sl].mean())
        print(f"  {d['t'][i]:7.1f} {(j-i)*DT:5.1f} {np.median(gap[sl]):7.1f} {gap[sl].max():7.1f} "
              f"{a[sl].mean():6.2f} {a[sl].min():6.2f} {np.percentile(a[sl],10):6.2f} "
              f"{(v[i]-v[j-1])*MPH:7.1f}")
    if runs:
        print(f"\n  transient peak decel: median {np.median(peaks):.2f}  best {min(peaks):.2f}")
        print(f"  run-mean decel:       median {np.median(means):.2f}  best {min(means):.2f}")

    # response vs gap, conditioned on the gap having been held
    print("\n=== steady-state response vs gap (gap held +-1 mph for >=1.5 s) ===")
    held = np.ones(len(gap), dtype=bool)
    for k in range(1, 31):
        held[k:] &= np.abs(gap[k:] - gap[:-k]) < 1.0
    m0 = no_lead & held
    print(f"  {'gap mph':>9} {'n':>6} {'a med':>7} {'a p10':>7} {'a p90':>7}")
    for lo in range(-1, 14):
        m = m0 & (gap >= lo) & (gap < lo + 1)
        if m.sum() < 15:
            continue
        print(f"  {lo:3}-{lo+1:<5} {m.sum():6} {np.median(a[m]):7.2f} "
              f"{np.percentile(a[m],10):7.2f} {np.percentile(a[m],90):7.2f}")

    # the marginal value of gap depth
    print("\n=== marginal value of extra gap ===")
    for lo, hi in ((3, 5), (5, 7), (7, 9), (9, 14)):
        m = m0 & (gap >= lo) & (gap < hi)
        if m.sum() < 15:
            continue
        print(f"  gap {lo:2}-{hi:<3}: n={m.sum():5}  median a = {np.median(a[m]):5.2f}  "
              f"p10 = {np.percentile(a[m],10):5.2f}")

    # latency: gap step -> decel response
    print("\n=== response latency: from the plan asking to the car decelerating ===")
    lim = np.isin(d['src'], ('sccVision', 'sccMap'))
    starts = np.where(lim[1:] & ~lim[:-1])[0] + 1
    lat_req, lat_dash, lat_move = [], [], []
    for s in starts:
        if not eng[s] or v[s] < 8:
            continue
        w = slice(s, min(len(v), s + 200))
        aa, gg = a[w], gap[w]
        if d['aT'][w].min() > -0.3:
            continue
        # t to first dash movement below vEgo by 1 mph
        i_gap = np.argmax(gg > 1.0) if (gg > 1.0).any() else None
        i_a03 = np.argmax(aa < -0.3) if (aa < -0.3).any() else None
        i_a05 = np.argmax(aa < -0.5) if (aa < -0.5).any() else None
        if i_gap is None or i_a03 is None:
            continue
        lat_dash.append(i_gap * DT)
        lat_req.append(i_a03 * DT)
        if i_a05 is not None and (aa < -0.5).any():
            lat_move.append(i_a05 * DT)
    if lat_req:
        print(f"  n={len(lat_req)} limiter engagements")
        print(f"  source flip -> dash 1 mph below vEgo : median {np.median(lat_dash):.2f} s")
        print(f"  source flip -> a < -0.3 m/s2         : median {np.median(lat_req):.2f} s")
        if lat_move:
            print(f"  source flip -> a < -0.5 m/s2         : median {np.median(lat_move):.2f} s  "
                  f"(n={len(lat_move)})")
        print(f"  limits.py assumes _STOCK_RESPONSE_T = 1.0 s plus the dash traversal")


if __name__ == '__main__':
    main()
