"""Cross-validate the far-field curvature under-read on routes other than 135.

The correction is a change to the planner for every car, so the bias it corrects has to be
a property of the model, not of one test road. Runs the same measurement over any other
local rlog set: model kappa at range around where an apex will be, versus the curvature
the car actually pulled there.
"""
import os, sys, glob
import numpy as np
from openpilot.tools.lib.logreader import LogReader

MPH = 2.23694
DT = 0.05
V_FLOOR = 0.5


def extract(paths):
    cs_t, cs_v, cs_eng = [], [], []
    ct_t, ct_k = [], []
    md_t, md_p = [], []
    for p in paths:
        try:
            lr = LogReader(p)
        except Exception as e:
            print(f'  skip {os.path.basename(p)}: {e}')
            continue
        for m in lr:
            t = m.logMonoTime / 1e9
            w = m.which()
            if w == 'carState':
                cs_t.append(t); cs_v.append(m.carState.vEgo)
                cs_eng.append(m.carState.cruiseState.enabled)
            elif w == 'controlsState':
                ct_t.append(t); ct_k.append(m.controlsState.curvature)
            elif w == 'modelV2':
                mm = m.modelV2
                rz = np.abs(np.asarray(mm.orientationRate.z, dtype=np.float32))
                vx = np.asarray(mm.velocity.x, dtype=np.float32)
                px = np.asarray(mm.position.x, dtype=np.float32)
                py = np.asarray(mm.position.y, dtype=np.float32)
                if len(rz) < 2:
                    continue
                md_t.append(t); md_p.append((rz, vx, px, py))
    if len(md_t) < 100:
        return None
    md_t = np.asarray(md_t)
    o = np.argsort(md_t)
    md_t = md_t[o]; md_p = [md_p[i] for i in o]
    cs_t, ct_t = np.asarray(cs_t), np.asarray(ct_t)
    oc = np.argsort(cs_t); cs_t = cs_t[oc]
    v = np.interp(md_t, cs_t, np.asarray(cs_v)[oc])
    eng = np.asarray(cs_eng)[oc][np.clip(np.searchsorted(cs_t, md_t, 'right') - 1, 0, len(cs_t) - 1)]
    okt = np.argsort(ct_t)
    k = np.interp(md_t, np.asarray(ct_t)[okt], np.abs(np.asarray(ct_k))[okt])
    return md_t, v, eng.astype(bool), np.convolve(k, np.ones(5) / 5, mode='same'), md_p


def kappa_dist(p):
    rz, vx, px, py = p
    kap = rz.astype(float) / np.maximum(vx.astype(float), V_FLOOR)
    dd = np.empty(len(px), dtype=float)
    dd[0] = 0.
    dd[1:] = np.cumsum(np.hypot(np.diff(px.astype(float)), np.diff(py.astype(float))))
    return kap, dd


def analyse(name, paths):
    r = extract(paths)
    if r is None:
        print(f"{name}: not enough model frames"); return None
    t, v, eng, k_meas, md_p = r
    dt = np.median(np.diff(t))
    lat = v ** 2 * k_meas
    ok = v > 8
    # apexes
    n = len(v)
    ks = np.convolve(k_meas, np.ones(5) / 5, mode='same')
    cand = [i for i in range(2, n - 2) if ok[i] and lat[i] > 1.2
            and ks[i] >= ks[max(0, i - 10):min(n, i + 11)].max()]
    apex = []
    for i in cand:
        if apex and i - apex[-1] < int(2.0 / dt):
            if lat[i] > lat[apex[-1]]:
                apex[-1] = i
            continue
        apex.append(i)
    if not apex:
        print(f"{name}: no apexes (frames={len(v)}, latmax={lat.max():.2f}, vmax={v.max():.1f})"); return None

    samples = []
    for i in apex:
        ka = k_meas[i]
        if ka < 2e-4:
            continue
        for back in range(1, int(10 / dt)):
            j = i - back
            if j < 0:
                continue
            d_apex = np.trapezoid(v[j:i + 1], dx=dt)
            kap, dd = kappa_dist(md_p[j])
            if dd[-1] < d_apex or d_apex < 3:
                continue
            w = (dd >= d_apex - 12) & (dd <= d_apex + 12)
            if w.any():
                samples.append((d_apex, float(kap[w].max()) / ka))
    if len(samples) < 100:
        print(f"{name}: only {len(samples)} samples"); return None
    s = np.array(samples)
    print(f"\n{name}: {len(apex)} apexes, {len(s)} samples, {(t[-1]-t[0]):.0f}s")
    print(f"  {'dist m':>10} {'n':>5} {'ratio':>7}")
    out = {}
    for lo, hi in ((0, 30), (30, 50), (50, 70), (70, 90), (90, 110), (110, 130), (130, 200)):
        m = (s[:, 0] >= lo) & (s[:, 0] < hi)
        if m.sum() < 10:
            continue
        out[(lo, hi)] = float(np.median(s[m, 1]))
        print(f"  {lo:4}-{hi:<5} {m.sum():5} {np.median(s[m,1]):7.2f}")
    return out


def main():
    here = os.path.dirname(__file__)
    targets = []
    for name in sys.argv[1:]:
        f = sorted(glob.glob(os.path.join(here, 'device_data', f'{name}--*', 'rlog*')))
        if not f:
            d = os.path.join(here, 'test_data', name)
            f = sorted(glob.glob(f'{d}/rlog*.zst')) or sorted(glob.glob(f'{d}/*/rlog.zst'))
        if f:
            targets.append((name, f))
        else:
            print(f'{name}: nothing found')
    print("far-field curvature under-read, other routes")
    print("(ratio = model kappa at that range / curvature actually pulled at the apex)")
    for name, f in targets:
        analyse(name, f)
    print("\nroute 135 for comparison:")
    print("     0-30   1.00 | 30-50  0.94 | 50-70  0.88 | 70-90  0.79")
    print("    90-110  0.66 | 110-130 0.55 | 130-200 0.30")


if __name__ == '__main__':
    main()
