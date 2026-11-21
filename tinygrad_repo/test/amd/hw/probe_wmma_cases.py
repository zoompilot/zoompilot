# Run caller-built v_wmma_f32_16x16x16_f16 cases on a gfx12 GPU (or the amd mock) and save the raw results.
#   DEV=USB+AMD python test/amd/hw/probe_wmma_cases.py IN.npz OUT.npz
# IN.npz holds "cases": (N, 32, 16) uint32, per lane the raw A v[0:3], B v[4:7] and C v[8:15] registers.
import sys, os, importlib.util
import numpy as np
from tinygrad.device import Device, Buffer
spec = importlib.util.spec_from_file_location("probe_rounding", os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_rounding.py"))
probe = importlib.util.module_from_spec(spec); spec.loader.exec_module(probe)

def run_cases(cases:np.ndarray) -> np.ndarray:
  probe.WMMA_CASES = len(cases)
  probe.wmma_inputs = lambda: cases
  return probe.probe_wmma()["wmma_f32_16x16x16_f16"]

if __name__ == "__main__":
  cases = np.load(sys.argv[1])["cases"]
  out = run_cases(cases)
  np.savez_compressed(sys.argv[2], device=np.array(Device.DEFAULT), arch=np.array(str(getattr(Device[Device.DEFAULT], "arch", ""))), out=out)
  print(f"wrote {sys.argv[2]}: {len(cases)} cases", flush=True)
