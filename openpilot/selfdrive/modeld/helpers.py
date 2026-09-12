import io
import json
import pickle
import shutil
import struct
import tempfile
from pathlib import Path

from openpilot.common.file_chunker import get_manifest_path
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'
TG_INPUT_DEVICES_PATH = MODELS_DIR / 'tg_input_devices.json'
CHESTNUT_POWERED_VOLTAGE = 5000
CHESTNUT_PCIE_READY = 0x78


def get_tg_input_devices(process_name: str, chestnut: bool):
  with open(TG_INPUT_DEVICES_PATH) as f:
    return json.load(f)[process_name]['default' if not chestnut else 'chestnut']

def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def dump_oob(obj, f):
  with tempfile.TemporaryFile(dir=".") as tmp:
    def buffer_callback(pb: pickle.PickleBuffer):
      m = pb.raw()
      tmp.write(struct.pack('<q', m.nbytes))
      tmp.write(m)
      pb.release() # keep peak ram at ~1 buffer
    stream = io.BytesIO()
    pickle.Pickler(stream, protocol=5, buffer_callback=buffer_callback).dump(obj)
    opcodes = stream.getvalue()
    f.write(struct.pack('<q', len(opcodes)))
    f.write(opcodes)
    tmp.seek(0)
    shutil.copyfileobj(tmp, f)

def load_oob(f):
  opcodes = f.read(struct.unpack('<q', f.read(8))[0])
  def buffers():
    while (h := f.read(8)):
      pb = pickle.PickleBuffer(bytearray(struct.unpack('<q', h)[0]))
      f.readinto(pb)
      yield pb
  return pickle.load(io.BytesIO(opcodes), buffers=buffers())

# the top level of the pkl compile_modeld.py writes. checked at both ends: a pkl from another
# compile_modeld.py unpickles fine and only fails on the first key it lacks, as a bare KeyError
MODELD_PKL_KEYS = ('metadata', 'input_devices', 'run_model')

def check_modeld_pkl(jits: dict, path) -> None:
  missing = [k for k in MODELD_PKL_KEYS if k not in jits]
  if missing:
    raise RuntimeError(f"{path} is missing {missing}: it was compiled by a different compile_modeld.py than this modeld, rebuild it")

# every warp jit is compiled for one driver camera resolution and keyed by it, so a prebuilt cut
# on a device with a different camera carries none for this one. Without this the mismatch
# surfaces as a bare FileNotFoundError or KeyError inside ModelState.__init__ and the process
# just stops. See PREBUILT_ALL_CAMERAS in modeld/SConscript
def _camera_mismatch(cam_w: int, cam_h: int, compiled: str, path) -> RuntimeError:
  have = f"{path} has only [{compiled or 'none'}]"
  return RuntimeError(f"no jit for this device's {cam_w}x{cam_h} driver camera: {have}. This build was compiled for another device")

def dm_warp_path(cam_w: int, cam_h: int):
  path = MODELS_DIR / f'dm_warp_{cam_w}x{cam_h}_tinygrad.pkl'
  if not path.is_file():
    compiled = ', '.join(sorted(p.name.split('_')[2] for p in MODELS_DIR.glob('dm_warp_*_tinygrad.pkl')))
    raise _camera_mismatch(cam_w, cam_h, compiled, MODELS_DIR)
  return path

def check_camera_jit(jits: dict, cam_w: int, cam_h: int, path) -> None:
  if (cam_w, cam_h) not in jits:
    compiled = ', '.join(f'{w}x{h}' for w, h in sorted(k for k in jits if isinstance(k, tuple)))
    raise _camera_mismatch(cam_w, cam_h, compiled, path)

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def chestnut_compiled() -> bool:
  return Path(get_manifest_path(modeld_pkl_path(chestnut=True))).is_file()


def chestnut_ready(state) -> bool:
  return state.supplyVoltage >= CHESTNUT_POWERED_VOLTAGE and not state.supplyFault and state.pcieLtssm == CHESTNUT_PCIE_READY
