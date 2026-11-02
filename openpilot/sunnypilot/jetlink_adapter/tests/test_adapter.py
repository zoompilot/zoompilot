"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The adapter against the real openpilot: jetlink tests its side against a fake
and installs no openpilot, so whatever jetlink relies on of this fork is
pinned here, where a sync that moves it fails.
"""
import contextlib
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from unittest import mock

import jetlink.openpilot as jl
from jetlink.openpilot import hooks
from jetlink.openpilot.interface import OwnerConfig, load_adapter
from jetlink.openpilot.settings import FileParams, Settings
from jetlink.openpilot.testing import check_adapter, check_fallback

from openpilot.cereal import messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.jetlink_adapter import KEYS

Adapter = jetlink_adapter._adapter_class()
ROOT = Path(__file__).resolve().parents[4]
# what the resident owner must never import: swaglog pulls numpy, capnp and
# zmq in to publish a log line and costs 28 MB, and params imports swaglog.
# Measured on the comma: the owner is 10 MB, the daemon that imported the
# world was 47.5 MB
HEAVY = ('numpy', 'capnp', 'zmq', 'cereal', 'msgq', 'tinygrad', 'openpilot.cereal', 'openpilot.common.params',
         'openpilot.common.swaglog')


def run_fresh(argv: list[str]) -> subprocess.CompletedProcess:
  # this runner's own path, so jetlink is found wherever it is checked out
  env = {**os.environ, 'PYTHONPATH': os.pathsep.join([str(ROOT), *sys.path])}
  return subprocess.run(argv, capture_output=True, text=True, env=env, cwd=str(ROOT), timeout=120)


class TestTheInterface(OpenpilotTestCase):
  def test_it_is_jetlinks_adapter_module(self):
    # every side of the interface, a top level of the standard library, and an
    # owner_config() that imports nothing heavy: everything the owner imports
    # runs for the whole drive
    self.assertEqual(check_adapter(jetlink_adapter, heavy=HEAVY), [])

  def test_the_api_and_the_setting_are_jetlinks(self):
    self.assertEqual(jetlink_adapter.API, jl.API)
    self.assertEqual(jetlink_adapter.MODES, jl.MODES)

  def test_the_states_are_the_ones_modeld_publishes(self):
    from openpilot.cereal import custom
    names = set(custom.ModelDataV2ZP.AcceleratorState.schema.enumerants)
    self.assertLessEqual(set(jl.STATES), names)

  def test_it_is_an_adapter_module(self):
    self.assertIsInstance(load_adapter(jetlink_adapter.__name__), Adapter)
    config = jetlink_adapter.owner_config()
    self.assertIsInstance(config, OwnerConfig)
    self.assertEqual(config.adapter, jetlink_adapter.__name__)
    self.assertEqual((config.cwd, dict(config.env)), (Path(BASEDIR), {'PYTHONPATH': BASEDIR}))
    self.assertEqual(config.keys, Adapter().keys)
    self.assertEqual(config.log_file, Path('/data/log/jetlink-owner.log'))


class TestParams(OpenpilotTestCase):
  def test_the_directory_is_params(self):
    # OpenpilotTestCase runs each test under a prefix of its own
    self.assertEqual(jetlink_adapter._params_dir(), Path(Params().get_param_path()))
    self.assertEqual(Adapter().params_dir(), jetlink_adapter._params_dir())

  def test_the_directory_follows_params_root_and_the_prefix(self):
    with tempfile.TemporaryDirectory() as root:
      for prefix in ('d', 'bench'):
        with mock.patch.dict(os.environ, {'PARAMS_ROOT': root, 'OPENPILOT_PREFIX': prefix}):
          self.assertEqual(jetlink_adapter._params_dir(), Path(Params().get_param_path()))
          self.assertEqual(jetlink_adapter._params_dir(), Path(root) / prefix)
          self.assertEqual(Adapter().params_dir(), Path(root) / prefix)

  def test_the_owner_reads_what_params_wrote(self):
    # jetlink reads the two hot settings as files, by params.cc's format
    params = Params()
    settings = Settings(FileParams(jetlink_adapter.owner_config().params_dir), Adapter().keys)
    for i, mode in enumerate(jetlink_adapter.MODES):
      params.put(KEYS.link, i, block=True)
      self.assertEqual(settings.mode(), mode)
    for parked in (True, False):
      params.put_bool(KEYS.offroad, parked, block=True)
      self.assertEqual(settings.offroad(), parked)
    for charge in (True, False):
      params.put_bool(KEYS.charge_phone, charge, block=True)
      self.assertEqual(settings.charge_phone(), charge)
    before = settings.marks()[KEYS.big_model]
    params.put(KEYS.big_model, {'ref': 'a' * 40, 'displayName': 'x'}, block=True)
    self.assertNotEqual(settings.marks()[KEYS.big_model], before)

  def test_every_key_is_declared_with_its_type(self):
    types = {'link': ParamKeyType.INT, 'offroad': ParamKeyType.BOOL, 'charge_phone': ParamKeyType.BOOL, 'select': ParamKeyType.STRING}
    params = Params()
    for field, key in KEYS._asdict().items():
      self.assertEqual(params.get_type(key), types.get(field, ParamKeyType.JSON), key)

  def test_the_model_managers_keys(self):
    from openpilot.sunnypilot.models.fetcher import ModelFetcher
    from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS, REQUIRED_JSON_VERSION
    self.assertEqual(KEYS.big_model, ACTIVE_BUNDLE_KEYS['chestnut'])
    self.assertEqual(KEYS.catalog, f"ModelManager_ModelsCache{ModelFetcher.MODEL_SOURCES['chestnut'][1]}")
    self.assertEqual(Adapter().catalog_selector, REQUIRED_JSON_VERSION)

  def test_values_round_trip_through_params(self):
    op = Adapter()
    op.put(KEYS.progress, {'stage': 'build', 'frac': 0.5, 'msg': ''}, block=True)
    self.assertEqual(Params().get(KEYS.progress), {'stage': 'build', 'frac': 0.5, 'msg': ''})
    self.assertEqual(op.get(KEYS.progress), {'stage': 'build', 'frac': 0.5, 'msg': ''})
    op.remove(KEYS.progress)
    self.assertIsNone(op.get(KEYS.progress))
    # a key this build does not declare reads as unset, as jetlink's readers expect
    self.assertIsNone(op.get('NotAParamAnyBuildHas'))

  def test_downloads_sit_under_the_model_managers_root(self):
    from openpilot.common.hardware.hw import Paths
    self.assertEqual(Adapter().model_root(), Path(Paths.model_root()))


class TestTheDevice(OpenpilotTestCase):
  def test_the_chestnut_ids_are_the_hardware_modules(self):
    from openpilot.common.hardware.usb import CHESTNUT_ROM_USB_IDS, CHESTNUT_USB_IDS
    self.assertEqual(Adapter.CHESTNUT_IDS, frozenset(CHESTNUT_USB_IDS + CHESTNUT_ROM_USB_IDS))

  def test_a_chestnut_is_modelds_answer(self):
    for fitted in (True, False):
      with mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=fitted):
        self.assertEqual(Adapter().chestnut_present(), fitted)

  def test_the_camera_is_this_devices_road_camera(self):
    from openpilot.common.hardware import HARDWARE
    from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
    from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
    for device, camera in (('tici', _ar_ox_fisheye), ('tizi', _ar_ox_fisheye), ('mici', _os_fisheye)):
      with mock.patch.object(HARDWARE, 'get_device_type', return_value=device):
        self.assertEqual(Adapter().camera(), (camera.width, camera.height, *MEDMODEL_INPUT_SIZE))

  def test_the_warp_is_stock_modelds(self):
    # jetlink's stock default: a plain TinyJit pickle scons builds for every camera, on every
    # build (modeld_v2's copy is sunnypilot's own SPTGPLN1 container, which jetlink cannot read)
    from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
    from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
    for camera in (_ar_ox_fisheye, _os_fisheye):
      path = Adapter().warp_path(camera.width, camera.height, *MEDMODEL_INPUT_SIZE)
      self.assertEqual(path, Path(BASEDIR) / 'openpilot/selfdrive/modeld/models' /
                       f'driving_warp_{camera.width}x{camera.height}_tinygrad.pkl')
    sconscript = (Path(BASEDIR) / 'openpilot/selfdrive/modeld/SConscript').read_text()
    self.assertIn('File(f"models/driving_warp_{cam_w}x{cam_h}_tinygrad.pkl")', sconscript)


class TestModeld(OpenpilotTestCase):
  def test_the_cable_counts_only_with_the_link_off(self):
    # jetlink docs/porting.md: a Jetson on the port reads as a chestnut to cable_connected()
    from openpilot.common.hardware import usb
    for cable, link, counted in ((False, False, False), (True, False, True), (True, True, False)):
      with mock.patch.object(usb, 'cable_connected', return_value=cable), \
           mock.patch.object(jetlink_adapter, 'link_on', return_value=link):
        self.assertEqual(jetlink_adapter.chestnut_cable(), counted)

  def test_every_modeld_asks_the_adapter_about_the_cable(self):
    for path in ('openpilot/selfdrive/modeld/modeld.py', 'openpilot/sunnypilot/modeld_v2/modeld.py'):
      source = (Path(BASEDIR) / path).read_text()
      self.assertIn('jetlink_adapter.chestnut_cable()', source, path)
      self.assertNotIn('cable_connected', source, path)

  def test_the_model_face_is_modelds(self):
    from openpilot.selfdrive.modeld import modeld
    from openpilot.selfdrive.modeld.constants import ModelConstants
    from openpilot.selfdrive.modeld.parse_model_outputs import Parser
    from openpilot.sunnypilot.modeld_v2.constants import ModelConstants as V2ModelConstants
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    face = Adapter().model_face()
    self.assertIsInstance(face.parser(), Parser)
    self.assertEqual(face.frame_size(1928, 1208), get_nv12_info(1928, 1208)[3])
    self.assertEqual(face.desire_len, ModelConstants.DESIRE_LEN)
    self.assertIs(face.constants, V2ModelConstants)
    self.assertEqual((face.lat_smooth_seconds, face.long_smooth_seconds), (modeld.LAT_SMOOTH_SECONDS, modeld.LONG_SMOOTH_SECONDS))

  def test_the_action_function_is_modelds_plus_blendful(self):
    import numpy as np
    from openpilot.cereal import log
    from openpilot.selfdrive.modeld import modeld
    # adding Blendful after stock's action function equals adding it before only
    # while stock leaves the curvature unsmoothed
    self.assertEqual(modeld.LAT_SMOOTH_SECONDS, 0.0)
    action = Adapter().model_face().get_action_from_model
    prev = log.ModelDataV2.Action(desiredCurvature=0.002, desiredAcceleration=0.1)
    for v_ego in (0.1, 20.0):
      output = {'action': np.array([[0.001 * max(1.0, v_ego) ** 2, 0.5]], dtype=np.float32)}
      stock = modeld.get_action_from_model(output, prev, 0.3, 0.3, v_ego)
      self.assertEqual(action(output, prev, 0.3, 0.3, v_ego).to_dict(), stock.to_dict(), 'stock modeld passes no callback')
      blendful = mock.Mock(return_value=0.0005)
      blended = action(output, prev, 0.3, 0.3, v_ego, blendful)
      blendful.assert_called_once_with(stock.desiredCurvature)
      self.assertAlmostEqual(blended.desiredCurvature, stock.desiredCurvature + (0.0005 if v_ego > modeld.MIN_LAT_CONTROL_SPEED else 0.0), places=6)
      self.assertEqual(blended.desiredAcceleration, stock.desiredAcceleration)

  def test_in_control_shuts_the_swap_window_on_anything_in_control_or_unknown(self):
    op = Adapter()
    self.assertIsNone(op._sm, 'a SubMaster only in the modeld that asks')
    self.assertIs(op.in_control(), True, 'nothing heard yet')
    sm = op._sm
    self.assertEqual(sm.services, list(Adapter.IN_CONTROL_SERVICES))
    sm.update = lambda timeout=0: None   # the messages are set by hand below

    def healthy():
      for service in sm.services:
        sm.data[service] = getattr(messaging.new_message(service), service)
        sm.alive[service] = sm.valid[service] = True

    for failed in sm.services:
      for check in (sm.alive, sm.valid):
        healthy()
        self.assertFalse(op.in_control())
        check[failed] = False
        self.assertTrue(op.in_control(), failed)
    healthy()
    sm['carControl'].enabled = True
    self.assertTrue(op.in_control())
    sm['carControl'].enabled = False
    # MADS engaged with its lateral paused (a stop, a blinker, the brake):
    # nothing steers, but MADS does again on its own, so no swap
    sm['carControlSP'].mads.enabled = True
    self.assertTrue(op.in_control())
    sm['carControlSP'].mads.enabled = False
    self.assertFalse(op.in_control())
    # a rule that fails on the frame thread holds the swap off, never modeld
    with mock.patch.object(messaging, 'SubMaster', side_effect=RuntimeError('no msgq')):
      self.assertIs(Adapter().in_control(), True)

  def test_telemetry_is_a_cloudlog_event(self):
    op = Adapter()
    with mock.patch.object(op.log, 'event') as event:
      op.event('jetlinkTelemetry', dead=False, rtt_ms=3.0)
    event.assert_called_once_with('jetlinkTelemetry', dead=False, rtt_ms=3.0)


class TestTheOwner(OpenpilotTestCase):
  def test_manager_runs_the_owner_with_this_fork_s_config(self):
    with mock.patch('jetlink.openpilot.owner.main') as run_owner:
      jetlink_adapter.main()
    run_owner.assert_called_once_with(jetlink_adapter.owner_config())

  def test_manager_runs_it_while_the_link_is_on_and_no_chestnut_is_fitted(self):
    from openpilot.system.manager.process_config import managed_processes
    owner = managed_processes[jetlink_adapter.OWNER]
    self.assertEqual(owner.module, jetlink_adapter.__name__)
    params = Params()
    with mock.patch.object(jetlink_adapter, '_hooks', None), \
         mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=False):
      for mode in jetlink_adapter.MODES:
        params.put(KEYS.link, jetlink_adapter.MODES.index(mode), block=True)
        for started in (False, True):
          self.assertEqual(owner.should_run(started, params, None), mode != 'off', mode)
    # fresh hooks: jetlink holds a chestnut's answer for a while
    with mock.patch.object(jetlink_adapter, '_hooks', None), \
         mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=True):
      self.assertFalse(owner.should_run(False, params, None))

  def test_the_owners_provisioning_run_starts_on_this_adapter(self):
    # the argv jetlinkd starts, run for real: with the link off it reads the
    # setting through this adapter and is done
    from jetlink.openpilot.owner import worker
    Params().put(KEYS.link, 0, block=True)
    out = run_fresh(worker(jetlink_adapter.owner_config()))
    self.assertEqual(out.returncode, 0, out.stderr)


class FakeProc:
  """multiprocessing.Process as manager uses it, with no process behind it."""

  def __init__(self, name=None, target=None, args=()):
    self.name, self.args = name, args
    self.exitcode = None
    self.pid = 4242
    self.started = 0

  def start(self):
    self.started += 1

  def is_alive(self):
    return self.exitcode is None

  def join(self, timeout=None):
    pass


class TestManagerStartsADeadOwnerAgain(OpenpilotTestCase):
  """jetlinkd holds the gadget for as long as the link is on, so manager starts
  it again when it dies, where a plain PythonProcess leaves one that exited
  alone for good. jetlink's owner adopts what the dead one left and backs off a
  crash loop itself, so manager only has to start it, once per loop; the
  backoff for one that dies young is RestartingPythonProcess's, tested in
  system/manager/tests/test_processes.py."""

  def loops(self, proc, *, should_run=True):
    """manager's loop over this one process, with the Processes it makes, on
    a clock of its own (self.clock, seconds)."""
    from openpilot.system.manager import process
    made = []
    self.clock = 1000.0

    def make(**kwargs):
      made.append(FakeProc(**kwargs))
      return made[-1]
    patches = [mock.patch.object(process, 'Process', side_effect=make), mock.patch.object(proc, 'proc', None),
               mock.patch.object(proc, 'shutting_down', False),
               mock.patch.object(proc, 'should_run', lambda started, params, CP: should_run),
               mock.patch.object(proc, 'now', lambda: self.clock), mock.patch.object(proc, 'started_at', 0.0),
               mock.patch.object(proc, 'backoff', 0.0), mock.patch.object(proc, 'next_start', 0.0)]
    for patch in patches:
      patch.start()
      self.addCleanup(patch.stop)

    def loop(seconds: float = 0.5):
      process.ensure_running([proc], False, params=None, CP=None)
      self.clock += seconds
    return made, loop

  def owner(self):
    from openpilot.system.manager.process_config import managed_processes
    return managed_processes[jetlink_adapter.OWNER]

  def test_the_owner_is_started_again_once_a_loop_and_never_twice(self):
    owner = self.owner()
    made, loop = self.loops(owner)
    loop()
    self.assertEqual([p.started for p in made], [1])
    self.assertEqual(made[0].args, (jetlink_adapter.__name__, jetlink_adapter.OWNER))
    for _ in range(3):
      loop(60.0)   # alive: never a second one beside it
    self.assertEqual([p.started for p in made], [1])
    made[0].exitcode = 1   # it died, having run a while
    loop()
    self.assertEqual([p.started for p in made], [1, 1], 'the dead owner was not started again, or twice')
    self.assertIs(owner.proc, made[1])
    loop(60.0)
    self.assertEqual(len(made), 2)
    made[1].exitcode = -9   # killed, this time
    loop()
    loop()
    self.assertEqual([p.started for p in made], [1, 1, 1])

  def test_a_dead_owner_the_link_no_longer_wants_is_only_reaped(self):
    owner = self.owner()
    made, loop = self.loops(owner)
    loop()
    made[0].exitcode = 1
    owner.should_run = lambda started, params, CP: False
    loop()
    self.assertIsNone(owner.proc)
    self.assertEqual(len(made), 1)


class TestWithoutAUsableJetlink(OpenpilotTestCase):
  """The fork's guard: no checkout, a jetlink too old to have the hooks, or one
  of another API. A jetlink that binds and then fails is jetlink's to answer
  for (its tests/openpilot/test_hooks.py)."""

  def test_every_hook_answers_as_the_link_off(self):
    for name in jetlink_adapter.HOOK_NAMES:
      self.assertEqual(getattr(jetlink_adapter, name).__name__, name)
    for link in (0, 1):
      Params().put(KEYS.link, link, block=True)
      self.assertEqual(check_fallback(jetlink_adapter._Off('why'), jetlink_adapter.HOOK_NAMES,
                                      link_on=jetlink_adapter.link_on), [])

  def test_without_a_checkout_every_hook_is_the_link_off(self):
    # an empty jetlink_repo: manager, the UI, hardwared and modeld must not care
    code = '''
import sys
sys.modules['jetlink'] = None
from openpilot.common.params import Params
from openpilot.sunnypilot import jetlink_adapter as a
Params().put("JetlinkLink", 1, block=True)
c = {'bundles': []}
assert not a.should_run(False, None, None)
assert a.status() is None and a.reason() is None
assert not a.prepare(False) and a.attach('model', object(), 1, 1) == 'model'
assert not a.shutdown_step(False, 'test') and a.shutdown_step(True, 'test')
assert not a.should_extend_catalog() and a.extend_catalog(c) is c
assert a.model_state('f' * 40) is None and a.swap_events(0.01) is None
'''
    # under this test's prefix, which the child inherits
    out = run_fresh([sys.executable, '-c', code])
    self.assertEqual(out.returncode, 0, out.stderr)

  def test_another_api_or_one_too_old_turns_the_link_off_and_says_why(self):
    params = Params()
    another_api = [mock.patch.object(hooks, 'COMPATIBLE', frozenset({2})), mock.patch.object(hooks, 'API', 2)]
    too_old = [mock.patch.dict(sys.modules, {'jetlink.openpilot.hooks': None})]
    for patches, why in ((another_api, "jetlink package API 2, this build expects 3"),
                         (too_old, "jetlink package too old for this build")):
      with contextlib.ExitStack() as stack:
        for patch in (*patches, mock.patch.object(jetlink_adapter, '_hooks', None)):
          stack.enter_context(patch)
        params.put(KEYS.link, 1, block=True)
        # the setting is read as a file: nothing here builds a Params
        with mock.patch('openpilot.common.params.Params', side_effect=AssertionError('a Params was built')):
          # every hook is the fork's _Off, which the check above holds to the link off
          self.assertIsInstance(jetlink_adapter.hooks(), jetlink_adapter._Off)
          self.assertEqual(jetlink_adapter.reason(), why)
        # nobody who left the link off is told
        params.put(KEYS.link, 0, block=True)
        self.assertIsNone(jetlink_adapter.reason())
