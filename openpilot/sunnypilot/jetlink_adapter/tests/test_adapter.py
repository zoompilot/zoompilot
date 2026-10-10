"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The adapter against the real openpilot: jetlink tests its side against a fake
and installs no openpilot, so whatever jetlink relies on of this fork is
pinned here, where a sync that moves it fails.
"""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import jetlink.openpilot as jl
from jetlink.openpilot.interface import Openpilot, OwnerConfig, conformance, load_adapter
from jetlink.openpilot.settings import FileParams, Settings

from openpilot.cereal import messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.jetlink_adapter import KEYS, Adapter

ROOT = Path(__file__).resolve().parents[4]
# what the resident owner must never import: swaglog pulls numpy, capnp and
# zmq in to publish a log line and costs 28 MB, and params imports swaglog.
# Measured on the comma: the owner is 10 MB, the daemon that imported the
# world was 47.5 MB
HEAVY = ('numpy', 'capnp', 'zmq', 'cereal', 'msgq', 'tinygrad', 'openpilot.cereal', 'openpilot.common.params',
         'openpilot.common.swaglog')


def run_fresh(code: str) -> subprocess.CompletedProcess:
  # this runner's own path, so jetlink is found wherever it is checked out
  env = {**os.environ, 'PYTHONPATH': os.pathsep.join([str(ROOT), *sys.path])}
  return subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, env=env, cwd=str(ROOT), timeout=120)


def heavy(modules) -> list[str]:
  return sorted(m for m in modules if any(m == h or m.startswith(h + '.') for h in HEAVY))


class TestTheInterface(OpenpilotTestCase):
  def test_the_adapter_implements_it(self):
    self.assertEqual(conformance(Adapter(), Openpilot), [])

  def test_the_api_and_the_setting_are_jetlinks(self):
    self.assertEqual(jetlink_adapter.API, jl.API)
    self.assertEqual(jetlink_adapter.MODES, jl.MODES)

  def test_the_states_are_the_ones_modeld_publishes(self):
    from openpilot.cereal import custom
    names = set(custom.ModelDataV2SP.AcceleratorState.schema.enumerants)
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
    self.assertEqual(Adapter().params_dir(), jetlink_adapter.owner_config().params_dir)

  def test_the_directory_follows_params_root_and_the_prefix(self):
    with tempfile.TemporaryDirectory() as root:
      for prefix in ('d', 'bench'):
        with mock.patch.dict(os.environ, {'PARAMS_ROOT': root, 'OPENPILOT_PREFIX': prefix}):
          self.assertEqual(jetlink_adapter._params_dir(), Path(Params().get_param_path()))
          self.assertEqual(jetlink_adapter._params_dir(), Path(root) / prefix)

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
    types = {'link': ParamKeyType.INT, 'offroad': ParamKeyType.BOOL, 'charge_phone': ParamKeyType.BOOL}
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
    self.assertEqual(jetlink_adapter.CHESTNUT_IDS, frozenset(CHESTNUT_USB_IDS + CHESTNUT_ROM_USB_IDS))

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

  def test_the_warp_is_modelds_own_in_every_checkout(self):
    # modeld_tinygrad's, which the checkout carries (LFS) for every camera, so
    # a device without a build has it too
    from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
    from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
    for camera in (_ar_ox_fisheye, _os_fisheye):
      path = Adapter().warp_path(camera.width, camera.height, *MEDMODEL_INPUT_SIZE)
      self.assertEqual(path, Path(BASEDIR) / 'openpilot/sunnypilot/modeld_v2/models' /
                       f'driving_warp_{camera.width}x{camera.height}_tinygrad.pkl')
      self.assertTrue(path.is_file(), path)


class TestModeld(OpenpilotTestCase):
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
    self.assertIs(face.get_action_from_model, modeld.get_action_from_model)

  def test_in_control_shuts_the_swap_window_on_anything_in_control_or_unknown(self):
    op = Adapter()
    self.assertIsNone(op._sm, 'a SubMaster only in the modeld that asks')
    self.assertIs(op.in_control(), True, 'nothing heard yet')
    sm = op._sm
    self.assertEqual(sm.services, list(jetlink_adapter.IN_CONTROL))
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
    op._sm = None
    with mock.patch.object(messaging, 'SubMaster', side_effect=RuntimeError('no msgq')):
      self.assertIs(op.in_control(), True)

  def test_a_chestnut_never_asks_jetlink(self):
    with mock.patch.object(jetlink_adapter, '_hook', side_effect=AssertionError('asked')):
      self.assertFalse(jetlink_adapter.prepare(True))

  def test_attach_keeps_the_model_unless_the_link_joins(self):
    joined = object()
    with mock.patch.object(jetlink_adapter, '_hook', return_value=None):
      self.assertEqual(jetlink_adapter.attach('model', 'small', 1928, 1208), 'model')
    with mock.patch.object(jetlink_adapter, '_hook', return_value=joined) as hook:
      self.assertIs(jetlink_adapter.attach('model', 'small', 1928, 1208), joined)
    hook.assert_called_once_with('attach', 'small', 1928, 1208)

  def test_a_handover_is_the_count_moving_across_a_run(self):
    joining = SimpleNamespace(handovers=0)
    with mock.patch.object(jetlink_adapter, '_handovers', 0):
      self.assertFalse(jetlink_adapter.handed_over(object()), 'a plain ModelState never hands over')
      self.assertFalse(jetlink_adapter.handed_over(joining))
      joining.handovers = 1
      self.assertTrue(jetlink_adapter.handed_over(joining))
      self.assertFalse(jetlink_adapter.handed_over(joining))

  def test_the_state_is_the_joining_models(self):
    self.assertEqual(jetlink_adapter.state(object()), 'none')
    self.assertEqual(jetlink_adapter.state(SimpleNamespace(big_model_state='ready')), 'ready')

  def test_what_the_hooks_read_off_the_model_is_the_joining_models(self):
    from jetlink.openpilot.joining import JoiningModelState
    for name in ('handovers', 'big_model_state'):
      self.assertIsInstance(getattr(JoiningModelState, name), property, name)

  def test_telemetry_is_a_cloudlog_event(self):
    op = Adapter()
    with mock.patch.object(op.log, 'event') as event:
      op.event('jetlinkTelemetry', dead=False, rtt_ms=3.0)
    event.assert_called_once_with('jetlinkTelemetry', dead=False, rtt_ms=3.0)


class TestTheOwner(OpenpilotTestCase):
  def test_the_owners_path_stays_out_of_the_heavy_half(self):
    code = '''
import json, sys
from openpilot.sunnypilot import jetlink_adapter
import jetlink.openpilot.owner
jetlink_adapter.owner_config()
print(json.dumps(sorted(sys.modules)))
'''
    out = run_fresh(code)
    self.assertEqual(out.returncode, 0, out.stderr)
    self.assertEqual(heavy(json.loads(out.stdout)), [], 'everything the owner imports runs for the whole drive')

  def test_manager_runs_the_owner_with_this_fork_s_config(self):
    with mock.patch('jetlink.openpilot.owner.main') as run_owner:
      jetlink_adapter.main()
    run_owner.assert_called_once_with(jetlink_adapter.owner_config())

  def test_manager_runs_it_while_the_link_is_on_and_no_chestnut_is_fitted(self):
    from openpilot.system.manager.process_config import managed_processes
    owner = managed_processes[jetlink_adapter.OWNER]
    self.assertEqual(owner.module, jetlink_adapter.__name__)
    params = Params()
    with mock.patch.object(jetlink_adapter, '_bound', None), \
         mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=False):
      for mode in jetlink_adapter.MODES:
        params.put(KEYS.link, jetlink_adapter.MODES.index(mode), block=True)
        for started in (False, True):
          self.assertEqual(owner.should_run(started, params, None), mode != 'off', mode)
    with mock.patch.object(jetlink_adapter, '_bound', None), \
         mock.patch('openpilot.selfdrive.modeld.helpers.chestnut_present', return_value=True):
      self.assertFalse(owner.should_run(False, params, None))

  def test_the_owners_provisioning_run_starts_on_this_adapter(self):
    # the argv jetlinkd starts, run for real: with the link off it reads the
    # setting through this adapter and is done
    from jetlink.openpilot.owner import worker
    Params().put(KEYS.link, 0, block=True)
    env = {**os.environ, 'PYTHONPATH': os.pathsep.join([str(ROOT), *sys.path])}
    out = subprocess.run(worker(jetlink_adapter.owner_config()), capture_output=True, text=True, env=env, cwd=str(ROOT),
                         timeout=120)
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
  it again when it dies. A PythonProcess leaves one that exited alone for
  good; jetlink's owner adopts what the dead one left and backs off a crash
  loop itself, so manager only has to start it, once per loop."""

  def loops(self, proc, *, should_run=True):
    """manager's loop over this one process, with the Processes it makes, on
    a clock of its own (self.clock, seconds) where the wrapper has one."""
    from openpilot.system.manager import process
    made = []
    self.clock = 1000.0

    def make(**kwargs):
      made.append(FakeProc(**kwargs))
      return made[-1]
    patches = [mock.patch.object(process, 'Process', side_effect=make), mock.patch.object(proc, 'proc', None),
               mock.patch.object(proc, 'shutting_down', False),
               mock.patch.object(proc, 'should_run', lambda started, params, CP: should_run)]
    if hasattr(proc, 'now'):
      patches += [mock.patch.object(proc, 'now', lambda: self.clock), mock.patch.object(proc, 'started_at', 0.0),
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

  def test_one_that_dies_young_is_started_again_later_and_later(self):
    # it never reached jetlink's own backoff: an import error, or a raise
    # before the owner's loop. Twice a second, it would be a fork of manager
    # and a sentry report twice a second for the whole drive
    owner = self.owner()
    made, loop = self.loops(owner)
    for wait in (10.0, 20.0, 40.0):
      loop()
      made[-1].exitcode = 1
      born = len(made)
      loop(wait - 0.5)   # reaped; the next start waits
      loop()
      self.assertEqual(len(made), born, f'started again inside {wait:.0f} s')
      loop()
      self.assertEqual(len(made), born + 1, f'not started again after {wait:.0f} s')
      self.clock -= 0.5
    self.assertEqual(owner.backoff, 40.0)
    # one that runs a while resets it
    self.clock += 30.0
    made[-1].exitcode = 1
    loop()
    self.assertEqual(owner.backoff, 0.0)
    self.assertEqual(len(made), born + 2)

  def test_the_wait_is_capped(self):
    owner = self.owner()
    made, loop = self.loops(owner)
    loop()
    for _ in range(8):
      made[-1].exitcode = 1
      loop(owner.BACKOFF_MAX)
      loop()
    self.assertEqual(owner.backoff, owner.BACKOFF_MAX)

  def test_a_dead_owner_the_link_no_longer_wants_is_only_reaped(self):
    owner = self.owner()
    made, loop = self.loops(owner)
    loop()
    made[0].exitcode = 1
    owner.should_run = lambda started, params, CP: False
    loop()
    self.assertIsNone(owner.proc)
    self.assertEqual(len(made), 1)

  def test_a_plain_python_process_stays_dead(self):
    # why jetlinkd has the subclass: without it, a dead owner is the rest of
    # the boot without a link
    from openpilot.system.manager.process import PythonProcess
    plain = PythonProcess('plain', 'plain.module', lambda started, params, CP: True)
    made, loop = self.loops(plain)
    loop()
    made[0].exitcode = 1
    for _ in range(3):
      loop()
    self.assertEqual(len(made), 1)


class TestWithoutAUsableJetlink(OpenpilotTestCase):
  def _hooks(self) -> dict:
    catalog = {'bundles': []}
    return {
      'should_run': jetlink_adapter.should_run(False, None, None),
      'status': jetlink_adapter.status(),
      'reason': jetlink_adapter.reason(),
      'prepare': jetlink_adapter.prepare(False),
      'attach': jetlink_adapter.attach('model', object(), 1, 1),
      'request_shutdown': jetlink_adapter.request_shutdown('test'),
      'shutdown_pending': jetlink_adapter.shutdown_pending(),
      'should_extend_catalog': jetlink_adapter.should_extend_catalog(),
      'extend_catalog': jetlink_adapter.extend_catalog(catalog) is catalog,
      'model_state': jetlink_adapter.model_state('f' * 40),
    }

  NULL = {'should_run': False, 'status': None, 'reason': None, 'prepare': False, 'attach': 'model',
          'request_shutdown': False, 'shutdown_pending': False,
          'should_extend_catalog': False, 'extend_catalog': True, 'model_state': None}

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
assert not a.request_shutdown('test') and not a.shutdown_pending()
assert not a.should_extend_catalog() and a.extend_catalog(c) is c
assert a.model_state('f' * 40) is None
'''
    # under this test's prefix, which the child inherits
    out = run_fresh(code)
    self.assertEqual(out.returncode, 0, out.stderr)

  def test_another_api_turns_the_link_off_and_says_why(self):
    params = Params()
    for api, name, why in ((2, 'API', "jetlink package API 2, this build expects 3"),
                           (None, 'openpilot', "jetlink package too old for this build")):
      if name == 'API':
        patch = mock.patch.object(jl, 'API', api)
      else:
        patch = mock.patch.dict(sys.modules, {'jetlink.openpilot': None})
      with patch, mock.patch.object(jetlink_adapter, '_bound', None):
        params.put(KEYS.link, 1, block=True)
        # the setting is read as a file: nothing here builds a Params
        with mock.patch('openpilot.common.params.Params', side_effect=AssertionError('a Params was built')):
          self.assertEqual(self._hooks(), {**self.NULL, 'reason': why})
        # nobody who left the link off is told
        params.put(KEYS.link, 0, block=True)
        self.assertIsNone(jetlink_adapter.reason())

  def test_a_failing_jetlink_never_raises_into_its_caller(self):
    class Broken:
      def __getattr__(self, name):
        def fail(*args, **kwargs):
          raise RuntimeError(name)
        return fail
    with mock.patch.object(jetlink_adapter, '_bound', Broken()), mock.patch.object(jetlink_adapter, '_failed_hooks', {}):
      self.assertEqual(self._hooks(), self.NULL)

  def test_a_failure_is_logged_again_once_it_changes_or_has_cleared(self):
    outcomes = iter([RuntimeError('a'), RuntimeError('a'), RuntimeError('b'), None, RuntimeError('b')])

    class Flaky:
      def status(self):
        if (e := next(outcomes)) is not None:
          raise e
        return 'a snapshot'
    with mock.patch.object(jetlink_adapter, '_bound', Flaky()), mock.patch.object(jetlink_adapter, '_failed_hooks', {}), \
         mock.patch.object(jetlink_adapter, '_log_failure') as log:
      answers = [jetlink_adapter.status() for _ in range(5)]
    self.assertEqual(answers, [None, None, None, 'a snapshot', None])
    self.assertEqual([str(c.args[1]) for c in log.call_args_list], ['a', 'b', 'b'])
