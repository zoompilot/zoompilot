# Writes test/mockgpu/amd/gfx1200_verified.sigs: the rdna4 instruction signatures whose emulator results were bit-identical to a gfx1200
# gpu in every probe, from the hardware and emulator result files of probe_basic.py, probe_rounding.py and probe_wmma_cases.py.
#   python test/amd/hw/verify_gfx1200.py HW_DIR EMU_DIR
# each dir holds basic.npz, rounding.npz and wmma_*.npz written by the probes; a signature is listed only if all of its probes match
import sys, os, glob, importlib.util
import numpy as np
here = os.path.dirname(os.path.abspath(__file__))
def load(name):
  spec = importlib.util.spec_from_file_location(name, os.path.join(here, f"{name}.py")); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
  return m
pb = load("probe_basic")
import tinygrad.runtime.autogen.amd.rdna4.ins as r4
from tinygrad.renderer.amd.dsl import v

ROUNDING = {"rcp_f32": "v_rcp_f32_e32", "sqrt_f32": "v_sqrt_f32_e32", "exp_f32": "v_exp_f32_e32", "rsq_f32": "v_rsq_f32_e32", "log_f32": "v_log_f32_e32",
            "rcp_f16": "v_rcp_f16_e32", "exp_f16": "v_exp_f16_e32", "sqrt_f16": "v_sqrt_f16_e32", "rsq_f16": "v_rsq_f16_e32", "log_f16": "v_log_f16_e32",
            "fma_f16": "v_fma_f16", "fma_mixlo_f16": "v_fma_mixlo_f16", "cvt_f16_f32": "v_cvt_f16_f32_e32"}

def main(hw_dir, emu_dir):
  ok = pb.verified(os.path.join(hw_dir, "basic.npz"), os.path.join(emu_dir, "basic.npz"))
  bad = set()
  a, b = np.load(os.path.join(hw_dir, "rounding.npz")), np.load(os.path.join(emu_dir, "rounding.npz"))
  for k in a.files:
    if k in ("device", "arch"): continue
    if k == "wmma_f32_16x16x16_f16": continue
    sig = next((s for p, s in ROUNDING.items() if k == p or k.startswith(p + "_")), None)
    if sig is None: continue
    (ok.add if np.array_equal(a[k], b[k]) else bad.add)(sig)
  wmma_sig = pb.signature(r4.v_wmma_f32_16x16x16_f16(v[8:15], v[0:3], v[4:7], v[8:15]))
  wmma_ok = np.array_equal(a["wmma_f32_16x16x16_f16"], b["wmma_f32_16x16x16_f16"])
  for f in sorted(glob.glob(os.path.join(hw_dir, "wmma_*.npz"))):
    wmma_ok &= np.array_equal(np.load(f)["out"], np.load(os.path.join(emu_dir, os.path.basename(f)))["out"])
  (ok.add if wmma_ok else bad.add)(wmma_sig)
  ok -= bad
  out = os.path.join(here, "..", "..", "mockgpu", "amd", "gfx1200_verified.sigs")
  with open(out, "w") as f:
    f.write("# rdna4 instruction signatures the amd emulator reproduces bit for bit on a gfx1200 gpu; written by test/amd/hw/verify_gfx1200.py\n")
    f.write("".join(s + "\n" for s in sorted(ok)))
  print(f"{len(ok)} verified signatures -> {os.path.normpath(out)}; not verified: {sorted(bad)}")

if __name__ == "__main__": main(sys.argv[1], sys.argv[2])
