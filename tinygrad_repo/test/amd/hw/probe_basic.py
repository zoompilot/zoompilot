# Every float instruction the chestnut driving models use, on a gfx12 GPU and on the amd mock, over random and special inputs
# (normals over the whole exponent range, denormals, +-0, +-inf, quiet and signaling NaNs), in the float mode llvm gives model kernels.
#   DEV=USB+AMD python test/amd/hw/probe_basic.py run hw.npz ; DEV=MOCKUSB+AMD ... run emu.npz ; python ... compare hw.npz emu.npz
import sys, os, importlib.util
import numpy as np
spec = importlib.util.spec_from_file_location("probe_rounding", os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_rounding.py"))
probe = importlib.util.module_from_spec(spec); spec.loader.exec_module(probe)
from tinygrad.device import Device, Buffer
import tinygrad.runtime.autogen.amd.rdna4.ins as r4
from tinygrad.renderer.amd.dsl import s, v, ttmp, NULL

N, SEED = 1 << 16, 77

def f32_inputs(rng, n):
  kind = rng.integers(0, 10, n)
  e = np.where(kind < 4, rng.integers(1, 255, n), np.where(kind < 6, rng.integers(120, 134, n), 0))
  bits = (rng.integers(0, 2, n) << 31) | (e << 23) | rng.integers(0, 1 << 23, n)
  bits = np.where(kind == 6, (rng.integers(0, 2, n) << 31) | rng.integers(1, 1 << 23, n), bits)             # denormals
  bits = np.where(kind == 7, rng.choice([0, 0x80000000, 0x7F800000, 0xFF800000, 0x7FC00000, 0x7FA00001, 0xFFC00000, 1, 0x80000001, 0x7F7FFFFF], n), bits)
  bits = np.where(kind >= 8, rng.integers(0, 1 << 32, n), bits)                                              # raw bits
  return bits.astype(np.uint32)

def f16_inputs(rng, n):
  kind = rng.integers(0, 10, n)
  e = np.where(kind < 4, rng.integers(1, 31, n), np.where(kind < 6, rng.integers(10, 20, n), 0))
  bits = (rng.integers(0, 2, n) << 15) | (e << 10) | rng.integers(0, 1 << 10, n)
  bits = np.where(kind == 6, (rng.integers(0, 2, n) << 15) | rng.integers(1, 1 << 10, n), bits)
  bits = np.where(kind == 7, rng.choice([0, 0x8000, 0x7C00, 0xFC00, 0x7E00, 0x7D01, 0xFE00, 1, 0x8001, 0x7BFF], n), bits)
  bits = np.where(kind >= 8, rng.integers(0, 1 << 16, n), bits)
  return bits.astype(np.uint32)

def packed(rng, n): return f16_inputs(rng, n) | (f16_inputs(rng, n) << 16)
def ints(lo, hi): return lambda rng, n: rng.integers(lo, hi, n).astype(np.int64).astype(np.uint32)

# vector ops: dst v[3], sources v[2], v[4], v[5]; v[3] holds v[5] first for the accumulating (mac) forms
def vop(inst): return [inst]
def mac(inst): return [r4.v_mov_b32_e32(v[3], v[5]), inst]
def cmp32(inst): return [inst, r4.v_mov_b32_e32(v[6], 1), r4.v_cndmask_b32_e32(v[3], 0, v[6])]       # writes VCC
def cmp64(inst): return [inst, r4.v_cndmask_b32_e64(v[3], 0, 1, s[20])]
def mix(op, hi, sel=0): return [r4.VOP3P(op, vdst=v[3], src0=v[2], src1=v[4], src2=v[5], opsel=sel, opsel_hi=hi & 3, opsel_hi2=hi >> 2)]
K32, K16 = 0x3dcccccd, 0x2e66

TESTS = {
  # f32 arithmetic
  "v_add_f32_e32": (vop(r4.v_add_f32_e32(v[3], v[2], v[4])), (f32_inputs, f32_inputs)),
  "v_sub_f32_e32": (vop(r4.v_sub_f32_e32(v[3], v[2], v[4])), (f32_inputs, f32_inputs)),
  "v_subrev_f32_e32": (vop(r4.v_subrev_f32_e32(v[3], v[2], v[4])), (f32_inputs, f32_inputs)),
  "v_mul_f32_e32": (vop(r4.v_mul_f32_e32(v[3], v[2], v[4])), (f32_inputs, f32_inputs)),
  "v_fma_f32": (vop(r4.v_fma_f32(v[3], v[2], v[4], v[5])), (f32_inputs, f32_inputs, f32_inputs)),
  "v_fmac_f32_e32": (mac(r4.v_fmac_f32_e32(v[3], v[2], v[4])), (f32_inputs, f32_inputs, f32_inputs)),
  "v_fmac_f32_e64": (mac(r4.v_fmac_f32_e64(v[3], v[2], v[4])), (f32_inputs, f32_inputs, f32_inputs)),
  "v_fmaak_f32_e32": (vop(r4.v_fmaak_f32_e32(v[3], v[2], v[4], K32)), (f32_inputs, f32_inputs)),
  "v_fmamk_f32_e32": (vop(r4.v_fmamk_f32_e32(v[3], v[2], v[4], K32)), (f32_inputs, f32_inputs)),
  "v_max_num_f32_e32": (vop(r4.v_max_num_f32_e32(v[3], v[2], v[4])), (f32_inputs, f32_inputs)),
  "v_min_num_f32_e32": (vop(r4.v_min_num_f32_e32(v[3], v[2], v[4])), (f32_inputs, f32_inputs)),
  "v_min3_num_f32": (vop(r4.v_min3_num_f32(v[3], v[2], v[4], v[5])), (f32_inputs, f32_inputs, f32_inputs)),
  "v_max3_num_f32": (vop(r4.v_max3_num_f32(v[3], v[2], v[4], v[5])), (f32_inputs, f32_inputs, f32_inputs)),
  "v_ldexp_f32": (vop(r4.v_ldexp_f32(v[3], v[2], v[4])), (f32_inputs, ints(-300, 300))),
  "v_trunc_f32_e32": (vop(r4.v_trunc_f32_e32(v[3], v[2])), (f32_inputs,)),
  "v_rcp_f32_e32": (vop(r4.v_rcp_f32_e32(v[3], v[2])), (f32_inputs,)),
  "v_sqrt_f32_e32": (vop(r4.v_sqrt_f32_e32(v[3], v[2])), (f32_inputs,)),
  "v_exp_f32_e32": (vop(r4.v_exp_f32_e32(v[3], v[2])), (f32_inputs,)),
  # conversions
  "v_cvt_f16_f32_e32": (vop(r4.v_cvt_f16_f32_e32(v[3], v[2])), (f32_inputs,)),
  "v_cvt_f16_f32_e64": (vop(r4.v_cvt_f16_f32_e64(v[3], v[2])), (f32_inputs,)),
  "v_cvt_f32_f16_e32": (vop(r4.v_cvt_f32_f16_e32(v[3], v[2])), (f16_inputs,)),
  "v_cvt_f32_f16_e64": (vop(r4.v_cvt_f32_f16_e64(v[3], v[2])), (f16_inputs,)),
  "v_cvt_f16_u16_e32": (vop(r4.v_cvt_f16_u16_e32(v[3], v[2])), (ints(0, 1 << 16),)),
  "v_cvt_f32_i32_e32": (vop(r4.v_cvt_f32_i32_e32(v[3], v[2])), (ints(-(1 << 31), 1 << 31),)),
  "v_cvt_i32_f32_e32": (vop(r4.v_cvt_i32_f32_e32(v[3], v[2])), (f32_inputs,)),
  "v_cvt_u32_f32_e32": (vop(r4.v_cvt_u32_f32_e32(v[3], v[2])), (f32_inputs,)),
  # f16 arithmetic
  "v_add_f16_e32": (vop(r4.v_add_f16_e32(v[3], v[2], v[4])), (f16_inputs, f16_inputs)),
  "v_add_f16_e64": (vop(r4.v_add_f16_e64(v[3], v[2], v[4])), (f16_inputs, f16_inputs)),
  "v_sub_f16_e32": (vop(r4.v_sub_f16_e32(v[3], v[2], v[4])), (f16_inputs, f16_inputs)),
  "v_mul_f16_e32": (vop(r4.v_mul_f16_e32(v[3], v[2], v[4])), (f16_inputs, f16_inputs)),
  "v_mul_f16_e64": (vop(r4.v_mul_f16_e64(v[3], v[2], v[4])), (f16_inputs, f16_inputs)),
  "v_fma_f16": (vop(r4.v_fma_f16(v[3], v[2], v[4], v[5])), (f16_inputs, f16_inputs, f16_inputs)),
  "v_fmac_f16_e32": (mac(r4.v_fmac_f16_e32(v[3], v[2], v[4])), (f16_inputs, f16_inputs, f16_inputs)),
  "v_fmac_f16_e64": (mac(r4.v_fmac_f16_e64(v[3], v[2], v[4])), (f16_inputs, f16_inputs, f16_inputs)),
  "v_fmaak_f16_e32": (vop(r4.v_fmaak_f16_e32(v[3], v[2], v[4], K16)), (f16_inputs, f16_inputs)),
  "v_fmamk_f16_e32": (vop(r4.v_fmamk_f16_e32(v[3], v[2], v[4], K16)), (f16_inputs, f16_inputs)),
  "v_max_num_f16_e32": (vop(r4.v_max_num_f16_e32(v[3], v[2], v[4])), (f16_inputs, f16_inputs)),
  "v_rcp_f16_e32": (vop(r4.v_rcp_f16_e32(v[3], v[2])), (f16_inputs,)),
  "v_rcp_f16_e64": (vop(r4.v_rcp_f16_e64(v[3], v[2])), (f16_inputs,)),
  "v_exp_f16_e32": (vop(r4.v_exp_f16_e32(v[3], v[2])), (f16_inputs,)),
  "v_pack_b32_f16": (vop(r4.v_pack_b32_f16(v[3], v[2], v[4])), (f16_inputs, f16_inputs)),
  "v_pk_add_f16": (vop(r4.v_pk_add_f16(v[3], v[2], v[4])), (packed, packed)),
  "v_pk_mul_f16": (vop(r4.v_pk_mul_f16(v[3], v[2], v[4])), (packed, packed)),
  "v_pk_fma_f16": (vop(r4.v_pk_fma_f16(v[3], v[2], v[4], v[5])), (packed, packed, packed)),
  # mixed precision, every f16/f32 source selection
  **{f"v_fma_mix_f32_hi{hi}": (mix(r4.VOP3POp.V_FMA_MIX_F32, hi), (packed, packed, packed)) for hi in range(8)},
  **{f"v_fma_mixlo_f16_hi{hi}": (mix(r4.VOP3POp.V_FMA_MIXLO_F16, hi), (packed, packed, packed)) for hi in range(8)},
  **{f"v_fma_mixlo_f16_hi{hi}_sel7": (mix(r4.VOP3POp.V_FMA_MIXLO_F16, hi, 7), (packed, packed, packed)) for hi in (1, 3, 7)},
  # compares
  **{f"v_cmp_{c}_f32_e32": (cmp32(getattr(r4, f"v_cmp_{c}_f32_e32")(v[2], v[4])), (f32_inputs, f32_inputs)) for c in ("eq", "lt", "gt", "ge", "ngt", "nlt", "neq", "le")},
  **{f"v_cmp_{c}_f32_e64": (cmp64(getattr(r4, f"v_cmp_{c}_f32_e64")(s[20], v[2], v[4])), (f32_inputs, f32_inputs)) for c in ("eq", "lt", "gt", "ge", "ngt", "nlt")},
  **{f"v_cmp_{c}_f16_e32": (cmp32(getattr(r4, f"v_cmp_{c}_f16_e32")(v[2], v[4])), (f16_inputs, f16_inputs)) for c in ("gt", "lt", "eq")},
  "v_cmp_gt_f16_e64": (cmp64(r4.v_cmp_gt_f16_e64(s[20], v[2], v[4])), (f16_inputs, f16_inputs)),
  "v_cmp_class_f32_e64": (cmp64(r4.v_cmp_class_f32_e64(s[20], v[2], v[4])), (f32_inputs, ints(0, 1 << 10))),
}

# scalar float ops: one result per wave, inputs per workgroup
SCALAR = {
  "s_mul_f32": lambda: [r4.s_mul_f32(s[16], s[14], s[15])],
  "s_add_f32": lambda: [r4.s_add_f32(s[16], s[14], s[15])],
  "v_s_rcp_f32": lambda: [r4.v_s_rcp_f32(s[16], s[14])],
}

def run_scalar(name, body, a, b):
  n = len(a)
  out = Buffer(Device.DEFAULT, n * 32 * 4).ensure_allocated()
  ins = [probe.gpu(a), probe.gpu(b)]
  code = probe.PROLOGUE + [r4.s_lshl_b32(s[13], ttmp[9], 2), r4.s_load_b32(s[14], s[6:7], soffset=s[13]), r4.s_load_b32(s[15], s[8:9], soffset=s[13]),
                           r4.s_wait_kmcnt(simm16=0), *body(), r4.v_mov_b32_e32(v[3], s[16]),
                           r4.s_lshl_b32(s[12], ttmp[9], 5), r4.v_add_nc_u32_e32(v[1], s[12], v[0]), r4.v_lshlrev_b32_e32(v[10], 2, v[1]),
                           r4.global_store_b32(vaddr=v[10:11], saddr=s[4:5], vsrc=v[3])]
  probe.launch(name, code, n, 32, [out, *ins, ins[0]])
  return probe.readback(out)[::32]

MODS = ("clmp", "omod", "abs", "neg", "neg_hi", "opsel", "opsel_hi", "opsel_hi2")
def signature(inst) -> str:  # op + the modifier fields it encodes, as the emulator's verified list and the model inventory spell it
  from tinygrad.viz.serve import amd_decode
  d = list(amd_decode(inst.to_bytes() + r4.s_endpgm().to_bytes(), "rdna4").values())[0]
  return "|".join([d.op_name.lower()] + [f"{f}={getattr(d, f)}" for f in MODS if getattr(d, f, 0)])

def gens_for(op:str):
  if op.startswith(("v_pk_", "v_fma_mix")): return packed
  if op.startswith("v_cvt_f32_f16") or op.startswith("v_cvt_f16_u16") and False: return f16_inputs
  if op.startswith("v_cvt_f16_u16"): return ints(0, 1 << 16)
  if op.startswith("v_cvt_f32_i32"): return ints(-(1 << 31), 1 << 31)
  if op.startswith("v_cvt_f32_u32"): return ints(0, 1 << 32)
  if "f16" in op and not op.startswith("v_cvt_f16_f32"): return f16_inputs
  return f32_inputs

def from_signature(sig:str):
  # the probe for a model signature: dst v[3], sources v[2], v[4], v[5]; compares materialize their lane bit
  op, *fields = sig.split("|")
  mods = {k: int(v) for k, v in (f.split("=") for f in fields)}
  fn = getattr(r4, op, None)
  if fn is None: return None
  g = gens_for(op)
  if op.startswith("v_cmp_"):
    if op.endswith("_e32"): return cmp32(fn(v[2], v[4])), (g, ints(0, 1 << 10) if "class" in op else g)
    return cmp64(fn(s[20], v[2], v[4], **mods)), (g, ints(0, 1 << 10) if "class" in op else g)
  if "fmaak" in op or "fmamk" in op: return vop(fn(v[3], v[2], v[4], K16 if "f16" in op else K32)), (g, g)
  if op.startswith("v_ldexp"): return vop(fn(v[3], v[2], v[4], **mods)), (g, ints(-300, 300))
  if op.startswith("v_fma_mix"):  # every VOP3P field spelled out, the helper's defaults differ from llvm's encoding
    f = {k: mods.get(k, 0) for k in ("opsel", "opsel_hi", "opsel_hi2", "neg", "neg_hi", "clmp")}
    inst = r4.VOP3P(getattr(r4.VOP3POp, op.upper()), vdst=v[3], src0=v[2], src1=v[4], src2=v[5], **f)
    return (vop(inst), (g, g, g)) if signature(inst) == sig else None
  for n in (3, 2, 1):
    try: inst = fn(v[3], *[v[2], v[4], v[5]][:n], **mods)
    except Exception: continue
    if signature(inst) != sig: continue
    body = mac(inst) if "fmac" in op else vop(inst)
    return body, (g,) * n
  return None

def run(path, sig_file=None):
  rng, res = np.random.default_rng(SEED), {}
  for name, (body, gens) in TESTS.items():
    inputs = tuple(g(rng, N) for g in gens)
    res[name] = probe.run_elementwise(name, body, N, inputs=inputs)
  for name, body in SCALAR.items():
    res[name] = run_scalar(name, body, f32_inputs(rng, 4096), f32_inputs(rng, 4096))
  if sig_file:  # every signature the models use that the fixed tests above do not spell out
    import json
    known = {signature(body[-1] if not name.startswith("v_cmp") else body[0]) for name, (body, _) in TESTS.items()}
    for sig in sorted(json.load(open(sig_file))):
      if sig in known or sig in res: continue
      if sig.startswith("s_"):  # scalar float ops: one result per wave
        fn = getattr(r4, sig.split("|")[0], None)
        made = None
        for args in ((s[16], s[14], s[15]), (s[16], s[14])):
          try: inst = fn(*args)
          except Exception: continue
          if signature(inst) == sig: made = inst; break
        if made is None: print(f"no probe for {sig}", flush=True); continue
        g = gens_for(sig.split("|")[0].replace("s_", "v_", 1))
        res["sig:" + sig] = run_scalar(sig, lambda inst=made: [inst], g(rng, 4096), g(rng, 4096))
        continue
      made = from_signature(sig)
      if made is None: print(f"no probe for {sig}", flush=True); continue
      body, gens = made
      res["sig:" + sig] = probe.run_elementwise("sig", body, N, inputs=tuple(gg(rng, N) for gg in gens))
  np.savez_compressed(path, device=np.array(Device.DEFAULT), **res)
  print(f"wrote {path}: {len(res)} instructions", flush=True)

def verified(a_path, b_path) -> set[str]:  # signatures whose hardware and emulator results are identical
  a, b = np.load(a_path), np.load(b_path)
  out = set()
  for k in a.files:
    if k == "device" or k not in b.files or not np.array_equal(a[k], b[k]): continue
    if k.startswith("sig:"): out.add(k[4:])
    elif k in TESTS:
      body = TESTS[k][0]
      out.add(signature(body[0] if k.startswith("v_cmp") else body[-1]))
    elif k in SCALAR: out.add(signature(SCALAR[k]()[0]))
  return out

def compare(a_path, b_path):
  a, b = np.load(a_path), np.load(b_path)
  bad = 0
  for k in [f for f in a.files if f != "device"]:
    x, y = a[k], b[k]
    d = np.flatnonzero(x != y)
    if d.size:
      bad += 1
      print(f"{k:28s} {d.size} of {x.size} differ, e.g. {[(hex(int(x[i])), hex(int(y[i]))) for i in d[:3]]}")
  print("all instructions MATCH" if not bad else f"{bad} instructions differ")

if __name__ == "__main__":
  if sys.argv[1] == "run": run(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
  elif sys.argv[1] == "compare": compare(sys.argv[2], sys.argv[3])
  elif sys.argv[1] == "verified": print("\n".join(sorted(verified(sys.argv[2], sys.argv[3]))))
