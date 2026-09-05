"""Route 135 overview: engagement, SCC states, curve episodes, apex lat accel."""
import os, pickle
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
MPH = 2.23694


def load():
    with open(os.path.join(DIR, 'scc_cache.pkl'), 'rb') as f:
        return pickle.load(f)


def resample(segs):
    """Everything onto the model (20 Hz) clock."""
    cat = lambda name, key: np.concatenate([s[1][name][key] for s in segs if len(s[1][name]['t'])])
    t_md = cat('md', 't')
    order = np.argsort(t_md)
    t_md = t_md[order]
    t0 = t_md[0]

    md = {k: np.concatenate([s[1]['md'][k] for s in segs if len(s[1]['md']['t'])])[order]
          for k in ('rate_z', 'velx', 'posx', 'posy')}

    out = {'t': t_md - t0, 't_abs': t_md}

    def interp_from(name, key, kind='f'):
        t = cat(name, 't'); v = cat(name, key)
        o = np.argsort(t); t, v = t[o], v[o]
        if kind == 'f':
            return np.interp(t_md, t, v.astype(float))
        idx = np.searchsorted(t, t_md, side='right') - 1
        idx = np.clip(idx, 0, len(v) - 1)
        return v[idx]

    for k in ('v', 'a', 'set', 'accel_pedal'):
        out[k] = interp_from('cs', k)
    for k in ('eng', 'ss', 'gas', 'brake'):
        out[k] = interp_from('cs', k, 's').astype(bool)
    out['curv'] = interp_from('ct', 'curv')
    for k in ('vT', 'aT', 'sccVT', 'sccAT', 'latAcc', 'maxLatAcc', 'vAhead', 'mapVT', 'mapAT'):
        out[k] = interp_from('lp', k)
    for k in ('src', 'sccState', 'mapState'):
        out[k] = interp_from('lp', k, 's')
    for k in ('icbm', 'btn'):
        out[k] = interp_from('sd', k, 's')
    out['ax'] = interp_from('lpo', 'ax')
    out['lat_gps'] = interp_from('lpo', 'lat')
    out['lon_gps'] = interp_from('lpo', 'lon')
    out['leadD'] = interp_from('rd', 'leadD')
    out['leadV'] = interp_from('rd', 'leadV')
    out['lead'] = interp_from('rd', 'leadStat', 's').astype(bool)
    out['cc_accel'] = interp_from('cc', 'accel')
    out['longActive'] = interp_from('cc', 'longActive', 's').astype(bool)
    out['md'] = md
    return out


def main():
    segs = load()
    d = resample(segs)
    t, v, eng = d['t'], d['v'], d['eng']
    print(f"duration {t[-1]:.0f}s  frames {len(t)}")
    print(f"speed {v.min()*MPH:.0f}-{v.max()*MPH:.0f} mph")
    print(f"engaged {eng.sum()/len(eng)*100:.0f}%  longActive {d['longActive'].sum()/len(t)*100:.0f}%")
    print(f"set speed values (engaged): {sorted(set(np.round(d['set'][eng]*MPH).astype(int)))}")

    print("\n=== plan source histogram (engaged) ===")
    for s in sorted(set(d['src'][eng])):
        n = (d['src'][eng] == s).sum()
        print(f"  {s:22} {n:6}  {n/eng.sum()*100:5.1f}%")

    print("\n=== SCC vision state (engaged) ===")
    for s in sorted(set(d['sccState'][eng])):
        n = (d['sccState'][eng] == s).sum()
        print(f"  {s:22} {n:6}  {n/eng.sum()*100:5.1f}%")

    print("\n=== ICBM state (engaged) ===")
    for s in sorted(set(d['icbm'][eng])):
        n = (d['icbm'][eng] == s).sum()
        print(f"  {s:22} {n:6}  {n/eng.sum()*100:5.1f}%")

    print("\n=== ICBM sendButton (engaged) ===")
    for s in sorted(set(d['btn'][eng])):
        n = (d['btn'][eng] == s).sum()
        print(f"  {s:22} {n:6}  {n/eng.sum()*100:5.1f}%")

    lat = v ** 2 * np.abs(d['curv'])
    m = eng & (v > 5)
    print(f"\n=== lateral accel (engaged, v>5 m/s, n={m.sum()}) ===")
    for p in (50, 75, 90, 95, 99, 100):
        print(f"  p{p:3}: {np.percentile(lat[m], p):.2f}")
    for th in (2.0, 2.2, 2.5, 3.0):
        print(f"  frames > {th}: {(lat[m] > th).sum():5}  ({(lat[m]>th).sum()/m.sum()*100:5.1f}%)")

    np.savez_compressed(os.path.join(DIR, 'resampled.npz'),
                        **{k: val for k, val in d.items() if k != 'md'})
    with open(os.path.join(DIR, 'model_paths.pkl'), 'wb') as f:
        pickle.dump({'t': d['t'], **d['md']}, f)
    print("\nsaved resampled.npz + model_paths.pkl")


if __name__ == '__main__':
    main()
