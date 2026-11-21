# Full v_exp_f32 dump for the amd emulator: where a gfx12 GPU's exp2 differs from the correctly rounded result.
#   DEV=USB+AMD python test/amd/hw/probe_exp.py OUT.npz
# |x| >= 1 is 2^floor(x) * a function of the 23-bit fraction, one table per sign (checked here over every exponent up to 2^7);
# |x| < 1 keeps more fraction bits, so every input from 2^-24 to 1 is swept and only the inputs that are not correctly rounded are kept.
import sys, os, time, importlib.util
import numpy as np
spec = importlib.util.spec_from_file_location("probe_rounding", os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_rounding.py"))
probe = importlib.util.module_from_spec(spec); spec.loader.exec_module(probe)
from tinygrad.device import Device
import tinygrad.runtime.autogen.amd.rdna4.ins as r4
from tinygrad.renderer.amd.dsl import v

def sweep(sign:int, exponent:int) -> np.ndarray:  # int8 ulp delta of hw exp2 vs correctly rounded, for all 2^23 mantissas
  base, n = (sign << 31) | (exponent << 23), 1 << 23
  bits = probe.run_elementwise("exp_f32", [r4.v_exp_f32_e32(v[3], v[2])], n, base=base)
  x = (np.arange(n, dtype=np.uint64) + base).astype(np.uint32).view(np.float32).astype(np.float64)
  with np.errstate(all="ignore"): return probe.f32_delta(bits, np.exp2(x))

def frac_index(sign:int, exponent:int) -> np.ndarray:  # for |x| >= 1: index of frac(x) = x - floor(x) on the 2^-23 grid
  m = np.arange(1 << 23, dtype=np.float64)
  x = (1 + m / 2**23) * 2.0 ** (exponent - 127) * (-1 if sign else 1)
  f = x - np.floor(x)
  return np.round(f * 2**23).astype(np.int64)

if __name__ == "__main__":
  st, out = time.perf_counter(), {}
  tables = {}
  for sign in (0, 1):
    d = sweep(sign, 127)                       # |x| in [1, 2): every fraction on the 2^-23 grid
    t = np.zeros(1 << 23, np.int8); t[frac_index(sign, 127)] = d
    tables[sign] = t; out[f"frac_{'neg' if sign else 'pos'}"] = t
    for e in range(128, 134):                  # |x| in [2, 128): must follow the same table
      d = sweep(sign, e)
      bad = np.flatnonzero(d != t[frac_index(sign, e)])
      out[f"check_{sign}_{e}"] = bad.astype(np.uint32)
      print(f"sign {sign} exponent {e}: {bad.size} inputs off the fraction table", flush=True)
  for sign in (0, 1):
    for e in range(103, 127):                  # |x| in [2^-24, 1): kept as the exceptions to correct rounding
      d = sweep(sign, e)
      nz = np.flatnonzero(d)
      out[f"small_{sign}_{e}_idx"], out[f"small_{sign}_{e}_val"] = nz.astype(np.uint32), d[nz]
      print(f"sign {sign} exponent {e}: {nz.size} not correctly rounded, range [{d.min()}, {d.max()}]", flush=True)
  for sign in (0, 1):                          # tiny inputs, and the overflow / underflow edges, sampled
    for e in list(range(90, 103, 4)) + [134, 135]:
      d = sweep(sign, e); nz = np.flatnonzero(d)
      out[f"edge_{sign}_{e}_idx"], out[f"edge_{sign}_{e}_val"] = nz.astype(np.uint32), d[nz]
      print(f"edge sign {sign} exponent {e}: {nz.size} not correctly rounded, range [{d.min()}, {d.max()}]", flush=True)
  np.savez_compressed(sys.argv[1], device=np.array(Device.DEFAULT), **out)
  print(f"wrote {sys.argv[1]} in {time.perf_counter()-st:.0f}s, {os.path.getsize(sys.argv[1])} bytes", flush=True)
