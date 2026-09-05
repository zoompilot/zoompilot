"""Extract SCC / decel-overshoot timeseries from route 135 rlogs into an npz cache."""
import glob, os, sys, pickle
import numpy as np
from openpilot.tools.lib.logreader import LogReader

DIR = os.path.join(os.path.dirname(__file__), 'test_data', 'route_135')
OUT = os.path.join(DIR, 'scc_cache.pkl')

MDL_KEEP = 33  # model path points


def process(path, seg):
    cs = {k: [] for k in ('t', 'v', 'a', 'set', 'eng', 'ss', 'gas', 'brake', 'accel_pedal')}
    ct = {k: [] for k in ('t', 'curv')}
    lp = {k: [] for k in ('t', 'vT', 'aT', 'src', 'sccState', 'sccVT', 'sccAT', 'latAcc', 'maxLatAcc', 'vAhead', 'mapState', 'mapVT', 'mapAT')}
    sd = {k: [] for k in ('t', 'icbm', 'btn')}
    md = {k: [] for k in ('t', 'rate_z', 'velx', 'posx', 'posy')}
    lpo = {k: [] for k in ('t', 'ax', 'lat', 'lon', 'valid')}
    rd = {k: [] for k in ('t', 'leadD', 'leadV', 'leadStat')}
    cc = {k: [] for k in ('t', 'accel', 'longActive')}
    lg = {k: [] for k in ('t', 'lat', 'lon')}
    try:
        lr = LogReader(path)
    except Exception as e:
        print(f'  {seg}: {e}'); return None
    for m in lr:
        t = m.logMonoTime / 1e9
        w = m.which()
        if w == 'carState':
            c = m.carState
            cs['t'].append(t); cs['v'].append(c.vEgo); cs['a'].append(c.aEgo)
            cs['set'].append(c.cruiseState.speedCluster); cs['eng'].append(c.cruiseState.enabled)
            cs['ss'].append(c.standstill); cs['gas'].append(c.gasPressed); cs['brake'].append(c.brakePressed)
            cs['accel_pedal'].append(getattr(c, 'gas', 0.))
        elif w == 'controlsState':
            ct['t'].append(t); ct['curv'].append(m.controlsState.curvature)
        elif w == 'longitudinalPlanSP':
            a = m.longitudinalPlanSP
            lp['t'].append(t); lp['vT'].append(a.vTarget); lp['aT'].append(a.aTarget)
            lp['src'].append(str(a.longitudinalPlanSource))
            v = a.smartCruiseControl.vision
            lp['sccState'].append(str(v.state)); lp['sccVT'].append(v.vTarget); lp['sccAT'].append(v.aTarget)
            lp['latAcc'].append(v.currentLateralAccel); lp['maxLatAcc'].append(v.maxPredictedLateralAccel)
            lp['vAhead'].append(v.vAheadMin)
            mm = a.smartCruiseControl.map
            lp['mapState'].append(str(mm.state)); lp['mapVT'].append(mm.vTarget); lp['mapAT'].append(mm.aTarget)
        elif w == 'selfdriveStateSP':
            i = m.selfdriveStateSP.intelligentCruiseButtonManagement
            sd['t'].append(t); sd['icbm'].append(str(i.state)); sd['btn'].append(str(i.sendButton))
        elif w == 'modelV2':
            mm = m.modelV2
            rz = np.asarray(mm.orientationRate.z, dtype=np.float32)
            vx = np.asarray(mm.velocity.x, dtype=np.float32)
            px = np.asarray(mm.position.x, dtype=np.float32)
            py = np.asarray(mm.position.y, dtype=np.float32)
            if len(rz) < 2:
                continue
            md['t'].append(t); md['rate_z'].append(rz); md['velx'].append(vx)
            md['posx'].append(px); md['posy'].append(py)
        elif w == 'liveLocationKalman':
            k = m.liveLocationKalman
            lpo['t'].append(t)
            lpo['ax'].append(k.accelerationCalibrated.value[0] if len(k.accelerationCalibrated.value) else 0.)
            lpo['lat'].append(k.positionGeodetic.value[0] if len(k.positionGeodetic.value) else 0.)
            lpo['lon'].append(k.positionGeodetic.value[1] if len(k.positionGeodetic.value) else 0.)
            lpo['valid'].append(bool(k.positionGeodetic.valid))
        elif w == 'radarState':
            r = m.radarState.leadOne
            rd['t'].append(t); rd['leadD'].append(r.dRel); rd['leadV'].append(r.vLead)
            rd["leadStat"].append(bool(r.present))
        elif w == 'carControl':
            c = m.carControl
            cc['t'].append(t); cc['accel'].append(c.actuators.accel)
            cc['longActive'].append(c.longActive)

    out = {}
    for name, d in (('cs', cs), ('ct', ct), ('lp', lp), ('sd', sd), ('lpo', lpo), ('cc', cc), ('rd', rd)):
        out[name] = {k: (np.asarray(v) if k != 'src' else np.asarray(v)) for k, v in d.items()}
    out['md'] = {'t': np.asarray(md['t']),
                 'rate_z': np.asarray(md['rate_z'], dtype=object),
                 'velx': np.asarray(md['velx'], dtype=object),
                 'posx': np.asarray(md['posx'], dtype=object),
                 'posy': np.asarray(md['posy'], dtype=object)}
    print(f'  seg{seg}: cs={len(cs["t"])} lp={len(lp["t"])} md={len(md["t"])}')
    return out


def main():
    files = sorted(glob.glob(f'{DIR}/rlog_seg*.zst'), key=lambda p: int(p.split('seg')[-1].split('.')[0]))
    print(f'{len(files)} segments')
    segs = []
    for f in files:
        seg = int(f.split('seg')[-1].split('.')[0])
        r = process(f, seg)
        if r: segs.append((seg, r))
    with open(OUT, 'wb') as fh:
        pickle.dump(segs, fh)
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
