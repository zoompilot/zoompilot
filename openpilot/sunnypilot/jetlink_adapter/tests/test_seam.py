"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Where jetlink meets modeld, in both modelds: stock modeld and sunnypilot's
modeld_tinygrad, which a custom small bundle runs on.

A device with comma's chestnut fitted must behave exactly as it does on
develop. chestnut is native, at its own lines, and jetlink is two adapter
calls beside it that a chestnut device never gets a yes from. This reads the
modelds rather than importing them (that costs tinygrad, usb1 and a vision
stream), runs stock modeld's decide and load statements verbatim under fakes,
and pins the footprint: chestnut's statements are develop's byte for byte,
and each modeld reaches the adapter from two calls.

It also holds the joining model to whatever the loop reads and writes on the
model it runs: a read the joining model cannot answer, or a write it would
keep to itself, is found here rather than on the frame thread of a drive.
"""
import ast
import functools
import subprocess
import textwrap
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from jetlink.comma import gadget
from jetlink.openpilot.joining import JoiningModelState

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import jetlink_adapter

OPENPILOT = Path(__file__).resolve().parents[3]
MODELD = OPENPILOT / 'selfdrive' / 'modeld' / 'modeld.py'
MODELD_V2 = OPENPILOT / 'sunnypilot' / 'modeld_v2' / 'modeld.py'
HARDWARED = OPENPILOT / 'system' / 'hardware' / 'hardwared.py'

# the zoompilot parent carries comma's chestnut unmodified, so it is the
# baseline rather than any older upstream tag
BASELINE = 'develop'

# everything modeld may call on the adapter: prepare() before the process goes
# realtime, attach() once the camera is up. A name added here without a plan
# entry is a widened seam
HOOKS = {'prepare', 'attach'}
ADAPTER = 'jetlink_adapter'


def _main(src: str) -> ast.FunctionDef:
  return next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == 'main')


def _assigns(stmt: ast.stmt, name: str) -> bool:
  return isinstance(stmt, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in stmt.targets)


def _index(body: list[ast.stmt], pred, what: str) -> int:
  for i, stmt in enumerate(body):
    if pred(stmt):
      return i
  raise AssertionError(f"main() no longer has {what}; this test is describing a file that moved on")


def _tests_name(stmt: ast.stmt, name: str) -> bool:
  return isinstance(stmt, ast.If) and isinstance(stmt.test, ast.Name) and stmt.test.id == name


def _calls(node: ast.AST) -> list[tuple[int, str]]:
  """Every `jetlink_adapter.<attr>` under node, as (line, attr)."""
  return [(n.lineno, n.attr) for n in ast.walk(node)
          if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == ADAPTER]


def _prepares(stmt: ast.stmt) -> bool:
  return any(attr == 'prepare' for _, attr in _calls(stmt))


def _realtime_index(body: list[ast.stmt]) -> int:
  return _index(body, lambda s: isinstance(s, ast.Expr) and isinstance(s.value, ast.Call)
                and isinstance(s.value.func, ast.Name) and s.value.func.id == 'config_realtime_process',
                'the config_realtime_process call')


def _fallback(body: list[ast.stmt]) -> ast.ExceptHandler:
  """The frame loop's handler that demotes a failed chestnut to the small model."""
  return next(h for n in body for x in ast.walk(n) if isinstance(x, ast.Try)
              for h in x.handlers if any('ChestnutActive' in ast.dump(s) for s in ast.walk(h)))


def _run(src: str, stmts: list[ast.stmt]) -> str:
  """The verbatim source of a run of statements, dedented so it can be exec'd."""
  lines = src.splitlines()
  return textwrap.dedent('\n'.join(lines[stmts[0].lineno - 1:stmts[-1].end_lineno]))


def _frame_loop(body: list[ast.stmt]) -> list[ast.stmt]:
  """main()'s last loop, the one that runs the model on every frame."""
  return [s for s in body if isinstance(s, ast.While)][-1].body


def _lagging_line() -> float:
  """The frameDropPerc past which selfdrived raises modeldLagging."""
  tree = ast.parse((OPENPILOT / 'selfdrive' / 'selfdrived' / 'selfdrived.py').read_text())
  return next(n.comparators[0].value for n in ast.walk(tree) if isinstance(n, ast.Compare) and isinstance(n.ops[0], ast.Gt)
              and isinstance(n.left, ast.Attribute) and n.left.attr == 'frameDropPerc')


def _git_show(ref: str, path: str) -> str | None:
  """The file at a ref, or None off a checkout that has it. Never raises."""
  try:
    out = subprocess.run(['git', 'show', f'{ref}:{path}'], cwd=OPENPILOT.parent,
                         capture_output=True, text=True, timeout=30, check=False)
  except (OSError, subprocess.SubprocessError):
    return None
  return out.stdout if out.returncode == 0 else None


def _model_attributes(src: str) -> tuple[set[str], set[str]]:
  """What main() reads and writes on `model`, the model the loop runs: every
  `model.<attr>` load and store, and getattr(model, '<attr>', ...)."""
  loads, stores = set(), set()
  for n in ast.walk(_main(src)):
    if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == 'model':
      (stores if isinstance(n.ctx, ast.Store) else loads).add(n.attr)
    elif (isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'getattr' and len(n.args) >= 2
          and isinstance(n.args[0], ast.Name) and n.args[0].id == 'model' and isinstance(n.args[1], ast.Constant)):
      loads.add(n.args[1].value)
  return loads, stores


class FakeAdapter:
  """The adapter as modeld sees it, recording every call.

  prepare() answers yes by default, so a call that should never have happened
  fails twice over: on the recording, and on a link joining where a chestnut
  owns the drive. attach() joins only after a yes, as jetlink's does.
  """

  def __init__(self, prepare=None, joined_type=SimpleNamespace):
    self.calls: list[str] = []
    self._prepare = prepare if prepare is not None else (lambda: True)
    self._joined_type = joined_type
    self.prepared = False
    self.small = None
    self.joined = None

  def prepare(self) -> bool:
    self.calls.append('prepare')
    self.prepared = self._prepare()
    return self.prepared

  def attach(self, small, cam_w, cam_h):
    self.calls.append('attach')
    self.small = small
    if not self.prepared:
      return None
    self.joined = self._joined_type(chestnut=False, big_model_state='joining')
    return self.joined


class FakeParams:
  """Params for the block under test only. The real ones belong to a car."""

  def __init__(self):
    self.store: dict[str, bool] = {}

  def put_bool(self, key, value, block=False):
    self.store[key] = bool(value)

  def get_bool(self, key):
    return bool(self.store.get(key))

  def remove(self, key):
    self.store.pop(key, None)


class FakeMessaging:
  """One chestnutState and then silence, which is the poller wait's fast path."""

  def __init__(self):
    self.polled = 0

  # Capitalised because messaging's is.
  def Poller(self):
    return self

  def poll(self, timeout_ms):
    self.polled += 1
    return [object()] if self.polled == 1 else []

  def sub_sock(self, name, poller=None, conflate=False):
    return object()

  def recv_one_or_none(self, sock):
    return SimpleNamespace(valid=True, chestnutState=object())


class FakeModelState:
  def __init__(self, cam_w, cam_h, chestnut):
    self.chestnut = chestnut

  def warmup(self):
    pass


class ModeldSeam:
  """Stock modeld's own statements, lifted out of main() and runnable with no hardware."""

  def __init__(self):
    self.src = MODELD.read_text()
    self.body = _main(self.src).body

    decide_start = _index(self.body, lambda s: _assigns(s, 'chestnut_available'), 'the chestnut_available assignment')
    self.decide_end = _index(self.body, _prepares, 'the jetlink_adapter.prepare() call')
    self.decide = _run(self.src, self.body[decide_start:self.decide_end + 1])

    load_start = _index(self.body, lambda s: _assigns(s, 'model') and isinstance(s.value, ast.Constant) and s.value.value is None,
                        'the `model = None` that opens the load')
    load_end = _index(self.body, lambda s: isinstance(s, ast.Assert) and s.lineno > self.body[load_start].lineno,
                      'the `assert model is not None` that closes the load')
    self.load = _run(self.src, self.body[load_start:load_end + 1])

    publish = _index(self.body, lambda s: _assigns(s, 'chestnut_state'), 'the chestnut_state assignment')
    self.publish = _run(self.src, [self.body[publish]])

    self.realtime = _realtime_index(self.body)

  def decide_and_load(self, adapter, present: bool, compiled: bool, trained: bool) -> dict:
    """Run the three blocks in order, exactly as main() does, and hand back its locals."""
    env: dict[str, str] = {}
    scope = {
      'chestnut_present': lambda: present,
      'chestnut_compiled': lambda: compiled,
      'chestnut_ready': lambda state: trained,
      'messaging': FakeMessaging(),
      # A short deadline: a board that never trains would otherwise wait out
      # deviceState's real period for a message the fake stops sending.
      'SERVICE_LIST': {'deviceState': SimpleNamespace(frequency=20)},
      'time': time,
      'threading': threading,
      'os': SimpleNamespace(environ=env),
      'Params': FakeParams,
      ADAPTER: adapter,
      'cloudlog': SimpleNamespace(warning=lambda *a, **k: None, exception=lambda *a, **k: None),
      'ModelState': FakeModelState,
      'ChestnutState': lambda pm, chestnut: SimpleNamespace(send=lambda *a: None, big=chestnut),
      'BIG_MODEL_TIMEOUT': 5,
      'vipc_client_main': SimpleNamespace(width=1928, height=1208),
      'pm': object(),
    }
    for block in (self.decide, self.load, self.publish):
      # in a function: the chestnut load declares `nonlocal big_model`. What
      # each block binds becomes a global for the next, which is how CHESTNUT
      # reaches the load
      wrapped = 'def _block():\n' + textwrap.indent(block, '  ') + '\n  return locals()\n'
      # exec of modeld's own source is the point of this file.
      exec(compile(wrapped, str(MODELD), 'exec'), scope)
      scope.update(scope.pop('_block')())
    scope['environ'] = env
    return scope


class NativeEquivalence(OpenpilotTestCase):
  """What stock modeld asks, for the configurations that must not join."""

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    cls.seam = ModeldSeam()

  def test_a_trained_chestnut_is_never_prepared(self):
    adapter = FakeAdapter()
    scope = self.seam.decide_and_load(adapter, present=True, compiled=True, trained=True)

    self.assertTrue(scope['CHESTNUT'])
    # prepare() is the only call that does anything, and `if not CHESTNUT`
    # guards it; attach() then answers None, as jetlink's does unprepared
    self.assertEqual(adapter.calls, ['attach'], "a fitted chestnut was prepared for the link")
    self.assertEqual(scope['environ'].get('HCQDEV_WAIT_TIMEOUT_MS'), '3000')
    self.assertTrue(scope['model'].chestnut)
    self.assertIsNotNone(scope['chestnut_state'])

  def test_no_board_and_the_link_off_stops_at_prepare(self):
    # the real adapter with the link off: prepare() reads one param and nothing
    # opens a gadget
    Params().put(jetlink_adapter.KEYS.link, jetlink_adapter.MODES.index('off'), block=True)
    adapter = FakeAdapter(prepare=jetlink_adapter.prepare)
    with mock.patch.object(gadget, 'link_configured') as link_configured:
      scope = self.seam.decide_and_load(adapter, present=False, compiled=False, trained=False)
    link_configured.assert_not_called()

    self.assertFalse(scope['CHESTNUT'])
    self.assertEqual(adapter.calls, ['prepare', 'attach'])
    self.assertIsNone(adapter.joined, "the disabled link joined")
    # the small model drives, and nothing publishes a chestnut's state
    self.assertIs(scope['model'], scope['small_model'])
    self.assertIsNone(scope['chestnut_state'])

  def test_a_board_that_never_trains_falls_through_to_the_question(self):
    # a chestnut device does ask, once, when the board is fitted but PCIe never
    # trains inside the poller wait: CHESTNUT is false, same as a bare device
    adapter = FakeAdapter(prepare=lambda: False)
    scope = self.seam.decide_and_load(adapter, present=True, compiled=True, trained=False)

    self.assertFalse(scope['CHESTNUT'])
    self.assertEqual(adapter.calls, ['prepare', 'attach'])
    self.assertIs(scope['model'], scope['small_model'])

  def test_a_prepared_link_is_one_attach(self):
    # no board and the link on: the small model is built as develop builds it
    # and handed over, and what attach() answers is the model modeld runs
    adapter = FakeAdapter()
    scope = self.seam.decide_and_load(adapter, present=False, compiled=False, trained=False)

    self.assertFalse(scope['CHESTNUT'])
    self.assertEqual(adapter.calls, ['prepare', 'attach'])
    self.assertIsNotNone(adapter.small)
    self.assertFalse(adapter.small.chestnut)
    self.assertIs(scope['small_model'], adapter.small)
    self.assertIs(scope['model'], adapter.joined)
    # the joining model asks for its own telemetry; nothing publishes a chestnut's state
    self.assertIsNone(scope['chestnut_state'])

  def test_the_joined_model_runs_whatever_its_truth(self):
    # a model whose __len__ is 0 is falsy, and `attach(...) or model` ran the small one instead
    class Empty(SimpleNamespace):
      def __len__(self):
        return 0
    adapter = FakeAdapter(joined_type=Empty)
    scope = self.seam.decide_and_load(adapter, present=False, compiled=False, trained=False)
    self.assertIs(scope['model'], adapter.joined)

  def test_the_link_is_decided_before_the_process_goes_realtime(self):
    # prepare() starts tinygrad's device thread; after config_realtime_process
    # it would inherit SCHED_FIFO 54 on core 7 and preempt the frame loop
    self.assertLess(self.seam.decide_end, self.seam.realtime,
                    "jetlink_adapter.prepare() moved after config_realtime_process")


class Footprint:
  """Shared by both modelds: two adapter calls, the decision before realtime,
  chestnut blocks that never reach the adapter, and the UI's field."""
  PATH: Path

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    cls.src = cls.PATH.read_text()
    cls.tree = ast.parse(cls.src)
    cls.body = _main(cls.src).body

  def test_the_adapter_is_reachable_from_two_calls_in_two_hunks(self):
    lines = self.src.splitlines()
    calls = sorted(_calls(self.tree))
    detail = '\n'.join(f"  {self.PATH.name}:{lineno} {lines[lineno - 1].strip()}" for lineno, _ in calls)
    self.assertEqual({attr for _, attr in calls}, HOOKS, f"the seam widened:\n{detail}")
    self.assertEqual(len(calls), 2, f"expected prepare and attach and nothing else:\n{detail}")
    # the decision and the load; the acceleratorState line reads the model
    self.assertGreater(calls[1][0] - calls[0][0], 8, f"one hunk where two were expected:\n{detail}")

  def test_the_link_is_decided_after_chestnut_and_before_realtime(self):
    decide = _index(self.body, _prepares, 'the jetlink_adapter.prepare() call')
    chestnut = _index(self.body, lambda s: _assigns(s, 'CHESTNUT'), 'the CHESTNUT assignment')
    self.assertLess(chestnut, decide)
    self.assertLess(decide, _realtime_index(self.body), "jetlink_adapter.prepare() moved after config_realtime_process")

  def test_a_chestnut_block_never_reaches_the_adapter(self):
    for stmt in ast.walk(self.tree):
      if _tests_name(stmt, 'CHESTNUT'):
        # the whole statement: the link attaches after the chestnut load, not as its `elif`
        names = {n.id for n in ast.walk(stmt) if isinstance(n, ast.Name)}
        self.assertNotIn(ADAPTER, names, f"an `if CHESTNUT:` block at line {stmt.lineno} reaches the adapter")

  def test_the_fallback_is_chestnuts_alone(self):
    # the joining model demotes itself and re-runs the frame; what reaches this
    # handler from it is a small-model fault, which ChestnutActive (never set
    # without a chestnut) re-raises, as develop does
    handler = _fallback(self.body)
    first = handler.body[0]
    self.assertTrue(isinstance(first, ast.If) and isinstance(first.body[0], ast.Raise)
                    and 'ChestnutActive' in ast.dump(first.test), "the fallback no longer opens with the ChestnutActive re-raise")
    self.assertNotIn(ADAPTER, {n.id for n in ast.walk(handler) if isinstance(n, ast.Name)})

  def test_the_ui_field_is_published(self):
    self.assertIn("modelDataV2SP.acceleratorState = getattr(model, 'big_model_state', 'none')", self.src)

  def _loop(self, frames: int, skipped, run) -> list[float]:
    """The loop's dropped-frame filter, its write of the share onto the model
    and its handover reset, verbatim, with `run(model, i)` as frame i's
    model.run() and `skipped(i)` camera frames dropped before frame i. Every
    frame's published frame_drop_ratio (frameDropPerc / 100)."""
    from openpilot.common.filter_simple import FirstOrderFilter
    loop = _frame_loop(self.body)
    first = _index(loop, lambda s: _assigns(s, 'vipc_dropped_frames'), 'the dropped-frame count')
    ratio = _index(loop, lambda s: _assigns(s, 'frame_drop_ratio'), 'frame_drop_ratio')
    write = _index(loop, lambda s: isinstance(s, ast.Assign) and any(isinstance(t, ast.Attribute) and t.attr == 'frame_drop_ratio'
                                                                       for t in s.targets), 'the write of the share onto the model')
    was = _index(loop, lambda s: _assigns(s, 'handovers'), 'the handover count read before run()')
    run_at = _index(loop, lambda s: isinstance(s, ast.Try) and 'run' in {n.attr for n in ast.walk(s) if isinstance(n, ast.Attribute)},
                    'the try around model.run')
    reset = _index(loop, lambda s: isinstance(s, ast.If) and 'handovers' in ast.dump(s.test), 'the handover reset')
    self.assertEqual((ratio < write, write), (True, was - 1), "the share is written onto the model just before run()")
    self.assertEqual(run_at, was + 2, "more than the timer between reading the model and running it")
    self.assertLess(run_at, reset)
    lines = self.src.splitlines()

    def src(a, b):
      return lines[loop[a].lineno - 1:loop[b].end_lineno]
    body = src(first, ratio) + src(write, was) + ['    model.run()'] + src(reset, reset)
    frame = compile(textwrap.dedent('\n'.join(body)), str(self.PATH), 'exec')
    model = SimpleNamespace(chestnut=False, handovers=0)
    scope = {'frame_dropped_filter': FirstOrderFilter(0., 10., 0.05), 'run_count': 0, 'last_vipc_frame_id': 0,
             'model': model, 'max': max, 'min': min}
    ratios, frame_id = [], 0
    for i in range(frames):
      frame_id += 1 + skipped(i)
      scope['meta_main'] = SimpleNamespace(frame_id=frame_id)
      model.run = functools.partial(run, model, i)
      exec(frame, scope)
      ratios.append(scope['frame_drop_ratio'])
      scope['last_vipc_frame_id'] = frame_id
    return ratios

  def _frames(self, handover: bool, stall_at: int = 20, gap: int = 3, on_the_drops: bool = False) -> list[float]:
    """40 frames, frame `stall_at` taking `gap` camera frames too long. With
    `handover` that run() is a swap whose first large frame fails and
    demotes, as jetlink's joining model does: modelV2.big is the same before
    and after, and only the handover count moves. With `on_the_drops` the
    handover is in the next frame's run() instead, the one that counts the
    dropped frames: the joining model handing back on them."""
    at = stall_at + 1 if on_the_drops else stall_at

    def run(model, i):
      if handover and i == at:
        # swapped in and demoted, chestnut stays False; or demoted alone
        model.handovers += 1 if on_the_drops else 2
    return self._loop(40, lambda i: gap if i == stall_at + 1 else 0, run)

  def test_a_handover_inside_run_is_not_lag(self):
    # upstream forgives the stall of a chestnut's fallback (run_count = 0);
    # jetlink's handover happens inside run(), where that handler never sees
    # it, and the one long frame read as 4 to 16 s of modeldLagging
    self.assertEqual(max(self._frames(handover=True)), 0.)
    self.assertEqual(max(self._frames(handover=True, gap=10)), 0.)
    # the same stall with no handover is lag, as upstream counts it: over the
    # 1 % selfdrived raises modeldLagging at
    self.assertGreater(max(self._frames(handover=False)) * 100, _lagging_line())

  def test_a_hand_back_on_dropped_frames_is_not_lag(self):
    # the frame whose run() hands back counted the drops; published as they
    # were, they were the frameDropPerc it went out with
    for gap in (2, 3):
      self.assertEqual(max(self._frames(handover=True, gap=gap, on_the_drops=True)), 0.)

  def test_a_large_model_behind_modeld_hands_back_short_of_lagging(self):
    # jetlink's rule against this loop's own share: whatever the pattern of
    # dropped frames, nothing the loop publishes reaches selfdrived's line,
    # the frame that hands back included
    from jetlink.openpilot.joining import DROP_LIMIT

    def run(model, i):
      if model.frame_drop_ratio > DROP_LIMIT:
        model.handovers += 1
    patterns = [(f'every {n}', lambda i, n=n: 1 if i % n == 0 else 0) for n in (1, 2, 5, 20, 60, 136, 200)]
    # a forgiven drop, then two at once
    patterns.append(('a double on a single', lambda i: {100: 1, 101: 2}.get(i, 0)))
    for name, skipped in patterns:
      with self.subTest(name):
        self.assertLess(max(self._loop(2000, skipped, run)) * 100, _lagging_line())

  def test_the_joining_model_answers_every_read_and_keeps_every_write(self):
    # read out of main() rather than kept by hand. A read the joining model
    # cannot answer is an AttributeError on the frame thread, which modeld
    # re-raises: no driving model for the drive, on jetlink devices only. A
    # write without a setter lands on the joining model and never reaches the
    # model that is driving
    loads, stores = _model_attributes(self.src)
    self.assertIn('run', loads, "this no longer finds what the loop reads off the model")
    self.assertEqual(sorted(n for n in loads | stores if n.startswith('_')), [])
    for name in stores:
      prop = getattr(JoiningModelState, name, None)
      self.assertTrue(isinstance(prop, property) and prop.fset is not None,
                      f"{self.PATH.name} writes model.{name}, which JoiningModelState keeps to itself")

    small = SimpleNamespace(**dict.fromkeys(loads | stores, 0.0))
    stop = threading.Event()

    def connect(should_stop=None):
      stop.wait(5)
      raise ConnectionError("no link in this test")

    def engagement():
      return lambda timeout_ms: stop.wait(timeout_ms / 1000) or True

    joining = JoiningModelState(small, connect, build=None, progress=mock.MagicMock(), engagement=engagement,
                                log=mock.MagicMock())
    try:
      missing = []
      for name in sorted(loads):
        try:
          getattr(joining, name)
        except AttributeError:
          missing.append(name)
      self.assertEqual(missing, [], f"JoiningModelState cannot answer {missing}, which {self.PATH.name} reads")
      for name in stores:
        setattr(joining, name, 1.0)
        self.assertEqual(getattr(small, name), 1.0, f"model.{name} = ... does not reach the small model")
    finally:
      stop.set()
      joining.close()


# every file outside the adapter that names it, and what each may use: the
# whole seam between this fork and jetlink. hardwared never asks for status():
# a snapshot keeps presence warm, and power-off would wait on a Jetson that
# had left seconds before. Nor for the blocking shutdown(): deviceState would
# stop for up to 25 s
SEAM = {
  'selfdrive/modeld/modeld.py': {'prepare', 'attach'},
  'sunnypilot/modeld_v2/modeld.py': {'prepare', 'attach'},
  'system/manager/process_config.py': {'OWNER', '__name__', 'should_run'},
  'sunnypilot/selfdrive/selfdrived/accelerator_events.py': {'OWNER'},
  'system/hardware/hardwared.py': {'reason', 'request_shutdown', 'shutdown_pending'},
  'sunnypilot/models/fetcher.py': {'should_extend_catalog', 'extend_catalog'},
  'selfdrive/ui/sunnypilot/ui_state.py': {'status'},
  'selfdrive/ui/sunnypilot/accelerator_link.py': {'KEYS', 'MODES'},
}


class TheWholeSeam(OpenpilotTestCase):
  def test_every_hook_uses_only_its_part_of_the_adapter(self):
    found = {}
    for path in sorted(OPENPILOT.rglob('*.py')):
      rel = path.relative_to(OPENPILOT).as_posix()
      if rel.startswith('sunnypilot/jetlink_adapter/') or '/tests/' in rel or b'jetlink_adapter' not in path.read_bytes():
        continue
      tree = ast.parse(path.read_bytes())
      used = {attr for _, attr in _calls(tree)}
      used |= {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module == 'openpilot.sunnypilot.jetlink_adapter'
               for a in n.names}
      found[rel] = used
    self.assertEqual(found, SEAM)


class FakeClock:
  def __init__(self):
    self.now = 100.0

  def monotonic(self) -> float:
    return self.now


class FakePowerOff:
  """The adapter's two power-off hooks, as hardwared sees them."""

  def __init__(self, asks: bool):
    self.asks = asks
    self.pending = asks
    self.requests: list[str] = []

  def request_shutdown(self, reason: str) -> bool:
    self.requests.append(reason)
    return self.asks

  def shutdown_pending(self) -> bool:
    return self.pending


class HardwaredPowersOffWithoutStopping(OpenpilotTestCase):
  """hardwared's shutdown check, lifted out of hardware_thread verbatim and run
  once per loop as the thread runs it, with the start of a drive before it.
  It asks jetlink once, goes on to publish deviceState every loop, and puts
  DoShutdown once the request is taken or 25 s have passed. The blocking
  shutdown() held the whole loop, and deviceState with it, for up to those
  25 s, and with it any drive starting; now a startup condition holds a new
  drive back instead."""

  def setUp(self):
    src = HARDWARED.read_text()
    fn = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == 'hardware_thread')
    init = next(n for n in fn.body if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)
                and n.target.id == 'accelerator_off_ts')
    loop = next(n for n in fn.body if isinstance(n, ast.While))
    body = loop.body
    powering_off = next(n for n in body if isinstance(n, ast.Assign) and 'not_powering_off' in ast.dump(n.targets[0]))
    should = next(i for i, n in enumerate(body) if _assigns(n, 'should_start') and 'all' in ast.dump(n.value))
    start = next(n for n in body if isinstance(n, ast.If) and isinstance(n.test, ast.Name) and n.test.id == 'should_start')
    check = next(n for n in body if isinstance(n, ast.If) and 'should_shutdown' in ast.dump(n.test))
    publish = next(i for i, n in enumerate(body) if "'deviceState'" in ast.dump(n) and 'send' in ast.dump(n))
    self.assertLess(body.index(powering_off), should, "the startup condition comes after the start it holds back")
    self.assertLess(body.index(check), publish, "deviceState is no longer published after the shutdown check")

    def code(*nodes):
      return compile('\n'.join(textwrap.dedent(ast.get_source_segment(src, n, padded=True)) for n in nodes),
                     str(HARDWARED), 'exec')
    self.init = code(init)
    self.loop = code(powering_off, body[should], body[should + 1], start, check)

  def run_loops(self, n: int, asks: bool = True, should_shutdown=True, clock_step: float = 0.5,
                ignition=lambda now: False, started_ts=None) -> SimpleNamespace:
    clock = FakeClock()
    jetlink = FakePowerOff(asks)
    params = FakeParams()
    onroad_conditions = {'ignition': False, 'device_temp_good': True}
    ns = {'power_monitor': SimpleNamespace(should_shutdown=lambda *a: should_shutdown(clock.now) if callable(should_shutdown)
                                           else should_shutdown),
          'onroad_conditions': onroad_conditions, 'startup_conditions': {'device_booted': True},
          'startup_conditions_prev': {}, 'startup_blocked_ts': None, 'started_ts': started_ts, 'in_car': True,
          'off_ts': 12.0 if started_ts is None else None, 'started_seen': True, 'cloudlog': mock.Mock(),
          'jetlink_adapter': jetlink, 'time': clock, 'params': params}
    exec(self.init, ns)
    self.assertIsNone(ns['accelerator_off_ts'])
    down_at, started = [], []
    for _ in range(n):
      onroad_conditions['ignition'] = ignition(clock.now)
      t0 = time.monotonic()
      exec(self.loop, ns)
      self.assertLess(time.monotonic() - t0, 0.1, 'the loop was held up')
      if params.get_bool('DoShutdown') and not down_at:
        down_at.append(clock.now)
      started.append(ns['started_ts'] is not None)
      clock.now += clock_step
    return SimpleNamespace(jetlink=jetlink, params=params, down_at=down_at[0] if down_at else None, ns=ns, clock=clock,
                           started=started)

  def test_no_drive_starts_while_it_powers_off(self):
    # the key turned inside the wait: a drive started now would have the comma
    # power off under it, possibly engaged
    r = self.run_loops(60, ignition=lambda now: now >= 105.0)
    self.assertEqual(r.down_at, 125.0)
    self.assertFalse(any(r.started), 'a drive started during the power-off')
    self.assertIs(r.ns['startup_conditions']['not_powering_off'], False)
    r.ns['cloudlog'].event.assert_any_call('Startup blocked', startup_conditions=mock.ANY, onroad_conditions=mock.ANY,
                                           error=True)

  def test_nothing_to_power_off_blocks_nothing(self):
    r = self.run_loops(4, should_shutdown=False, ignition=lambda now: now >= 101.0)
    self.assertEqual(r.started, [False, False, True, True])

  def test_a_drive_under_way_is_left_as_it_was(self):
    # ForcePowerDown onroad: the startup conditions only hold back a start
    r = self.run_loops(4, ignition=lambda now: True, started_ts=50.0)
    self.assertEqual(r.started, [True] * 4)

  def test_nothing_to_ask_goes_down_at_once(self):
    r = self.run_loops(1, asks=False)
    self.assertEqual(r.jetlink.requests, ['comma shutting down, offroad since 12.0'])
    self.assertEqual(r.down_at, 100.0)

  def test_it_asks_once_and_goes_down_when_the_request_is_taken(self):
    r = self.run_loops(6)
    self.assertEqual(len(r.jetlink.requests), 1)
    self.assertIsNone(r.down_at)
    r.jetlink.pending = False   # the owner's run took it
    exec(self.loop, r.ns)
    self.assertTrue(r.params.get_bool('DoShutdown'))
    self.assertEqual(len(r.jetlink.requests), 1)

  def test_nobody_taking_it_costs_25_s_and_no_more(self):
    r = self.run_loops(60)
    self.assertEqual(len(r.jetlink.requests), 1)
    self.assertEqual(r.down_at, 125.0)

  def test_once_asked_it_goes_down_whatever_the_power_monitor_says_next(self):
    # as the blocking call did: the Jetson may already be off
    r = self.run_loops(60, should_shutdown=lambda now: now == 100.0)
    self.assertEqual(r.down_at, 125.0)

  def test_no_shutdown_asks_nothing(self):
    r = self.run_loops(4, should_shutdown=False)
    self.assertEqual(r.jetlink.requests, [])
    self.assertIsNone(r.down_at)


class StockModeld(Footprint, OpenpilotTestCase):
  PATH = MODELD

  def _baseline(self) -> str:
    src = _git_show(BASELINE, 'openpilot/selfdrive/modeld/modeld.py')
    if src is None:
      self.skipTest(f"local only: needs the {BASELINE} branch to compare with, and CI's shallow checkout has none")
    return src

  def test_chestnut_state_is_the_baselines(self):
    baseline = self._baseline()

    def cls(src):
      return next((n for n in ast.parse(src).body if isinstance(n, ast.ClassDef) and n.name == 'ChestnutState'), None)
    ours, theirs = cls(self.src), cls(baseline)
    self.assertIsNotNone(ours, "ChestnutState left modeld.py again")
    self.assertIsNotNone(theirs)
    self.assertEqual(ast.get_source_segment(self.src, ours), ast.get_source_segment(baseline, theirs),
                     f"class ChestnutState differs from {BASELINE}")

  def test_the_poller_wait_and_the_chestnut_load_are_the_baselines(self):
    baseline = self._baseline()
    body = _main(baseline).body

    def wait(b):
      return next(s for s in b if _tests_name(s, 'chestnut_available'))

    def load(b):
      # the chestnut load, then the small model and its fallback, as develop has them
      i = b.index(next(s for s in b if _tests_name(s, 'CHESTNUT') and len(s.body) > 2))
      return b[i:i + 3]

    self.assertEqual(_run(self.src, [wait(self.body)]), _run(baseline, [wait(body)]),
                     f"the chestnutState poller wait differs from {BASELINE}")
    # the whole statement, orelse included, and the small model after it: the
    # link attaches below them rather than as a branch of them
    self.assertEqual(_run(self.src, load(self.body)), _run(baseline, load(body)),
                     f"the `if CHESTNUT:` load or the small model differs from {BASELINE}")

  def test_the_fallback_is_the_baselines(self):
    baseline = self._baseline()
    self.assertEqual(_run(self.src, _fallback(self.body).body), _run(baseline, _fallback(_main(baseline).body).body),
                     f"the big-model fallback differs from {BASELINE}")


class ModeldTinygrad(Footprint, OpenpilotTestCase):
  PATH = MODELD_V2

  def test_both_modelds_join_with_the_same_lines(self):
    # jetlink's end of the join lives behind the adapter; what is left in
    # modeld must not drift apart between the two copies
    def lines(src):
      return [line.strip() for line in src.splitlines() if ADAPTER in line or 'accelerator' in line.lower()]
    self.assertEqual(lines(self.src), lines(MODELD.read_text()))

  def test_driving_model_data_says_which_model_drove(self):
    # the qlog's only model message: stock modeld copies modelV2.big into it
    # (fill_driving_model_data), and modeld_v2 fills its own
    self.assertIn("drivingdata_send.drivingModelData.big = model.chestnut", self.src)


if __name__ == '__main__':
  unittest.main()
