"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The warps jetlink_adapter/SConscript builds, and that only the build compiles one.

The cameras are modeld/SConscript's, exported to it: a source build's own, or
every one a prebuilt release installs on. Nothing compiles a missing warp at
runtime, so a source build shrugs off a failed compile (the offroad alert
says so) and a release fails on it. The SConscript runs here against a
stand-in for SCons, and the targets it declares are compared with what
modeld opens: the adapter's warp_path, which jetlink loads from.
"""
import ast
import contextlib
import os
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
from openpilot.sunnypilot import jetlink_adapter

SCONSCRIPT = Path(BASEDIR) / 'openpilot' / 'sunnypilot' / 'jetlink_adapter' / 'SConscript'
MODEL = (512, 256)
CAMERAS = [(c.width, c.height) for c in (_ar_ox_fisheye, _os_fisheye)]


class FakeNode:
  def __init__(self, root: str, path: str):
    self.abspath = os.path.join(root, path[1:]) if path.startswith('#') else path
    self.relpath = os.path.relpath(self.abspath, root)


class FakeEnv:
  """The parts of an SCons environment the SConscript touches."""

  def __init__(self, root: str, pythonpath: str, commands: list | None = None):
    self.root = root
    self.env = {'ENV': {'PYTHONPATH': pythonpath}}
    self.commands: list[tuple[str, str, dict]] = [] if commands is None else commands

  def __getitem__(self, key):
    return self.env[key]

  def Clone(self, **overrides):
    clone = FakeEnv(self.root, self.env['ENV']['PYTHONPATH'], self.commands)
    clone.env.update(overrides)
    return clone

  def Dir(self, path: str) -> FakeNode:
    return FakeNode(self.root, path)

  def Command(self, target, source, action):
    self.commands.append((target, action, self.env['ENV']))


def run_sconscript(camera_configs=CAMERAS, prebuilt: bool = False, arch: str = 'comma_arm64', jetlink: Path | None = None,
                   capture: bool = True, pythonpath: str = 'the/build/path', root: str | None = None) -> dict[str, tuple[str, dict]]:
  """{target: (command, its environment)} for every warp the SConscript declares,
  in a checkout at `root` (a temporary one by default). Its jetlink_repo is
  `jetlink`, else an empty one with the capture in it or not."""
  with (contextlib.nullcontext(root) if root is not None else tempfile.TemporaryDirectory()) as checkout:
    (Path(checkout) / 'openpilot').symlink_to(Path(BASEDIR) / 'openpilot')
    if jetlink is not None:
      (Path(checkout) / 'jetlink_repo').symlink_to(jetlink)
    else:
      (Path(checkout) / 'jetlink_repo' / 'jetlink' / 'openpilot').mkdir(parents=True)
      if capture:
        (Path(checkout) / 'jetlink_repo' / 'jetlink' / 'openpilot' / 'warp.py').touch()
    (Path(checkout) / 'tinygrad_repo').mkdir()

    env = FakeEnv(checkout, pythonpath)
    script = types.ModuleType('SCons.Script')
    script.Action = lambda cmd, msg=None: cmd
    scons = types.ModuleType('SCons')
    scons.Script = script
    namespace = {'Import': lambda *names: None, 'env': env, 'arch': arch, 'camera_configs': camera_configs,
                 'Dir': env.Dir, 'File': env.Dir}

    with mock.patch.dict(sys.modules, {'SCons': scons, 'SCons.Script': script}), mock.patch.dict(os.environ):
      os.environ.pop('PREBUILT_ALL_CAMERAS', None)
      if prebuilt:
        os.environ['PREBUILT_ALL_CAMERAS'] = '1'
      exec(compile(SCONSCRIPT.read_text(), str(SCONSCRIPT), 'exec'), namespace)
    return {target: (cmd, cmd_env) for target, cmd, cmd_env in env.commands}


class TestWarpTargets(OpenpilotTestCase):
  def test_every_camera_modeld_builds_for_gets_a_warp_where_modeld_opens_it(self):
    targets = run_sconscript()
    op = jetlink_adapter.Adapter()
    self.assertEqual(set(targets), {str(op.warp_path(w, h, *MODEL)) for w, h in CAMERAS})
    for target in targets:
      # in the fork's tree: a file in the submodule leaves it dirty for the updater
      self.assertEqual(Path(target).parent, Path(BASEDIR) / 'openpilot' / 'sunnypilot' / 'jetlink_adapter' / 'models')

  def test_each_command_runs_jetlinks_build_for_the_camera_its_target_names(self):
    for target, (cmd, _) in run_sconscript().items():
      cam = Path(target).name.split('_')[1]
      self.assertIn(f'-m jetlink.openpilot.warp --adapter {jetlink_adapter.__name__} ', cmd)
      self.assertIn(f'--camera {cam} ', cmd)
      self.assertIn(f'--model {MODEL[0]}x{MODEL[1]} ', cmd)
      self.assertTrue(cmd.endswith(f'--output {target}'), cmd)

  def test_jetlink_is_on_the_commands_path_by_its_checkout(self):
    # the prebuilt runner makes no `jetlink` link at the root, so the path has
    # to name the submodule; the build's own path stays first
    for _, env in run_sconscript().values():
      path = env['PYTHONPATH'].split(os.pathsep)
      self.assertEqual(path[0], 'the/build/path')
      self.assertEqual(Path(path[-1]).name, 'jetlink_repo')

  def test_a_failed_compile_fails_a_release_but_not_a_source_build(self):
    # a device building from source stays on the small model and tries again
    # on the next build; a release installs on devices that never build
    for cmd, _ in run_sconscript(CAMERAS[:1]).values():
      self.assertTrue(cmd.startswith('-'), cmd)
    for cmd, _ in run_sconscript(prebuilt=True).values():
      self.assertFalse(cmd.startswith('-'), cmd)

  def test_nothing_is_built_off_the_comma(self):
    self.assertEqual(run_sconscript(arch='Darwin'), {})

  def test_nothing_is_built_without_a_jetlink_that_has_the_capture(self):
    # an empty jetlink_repo, or one from before jetlink.openpilot: a missing
    # source would fail the whole build, not just the warp
    self.assertEqual(run_sconscript(capture=False), {})


class TestTheCommandBuilds(OpenpilotTestCase):
  def test_the_command_builds_a_warp_with_nothing_but_its_own_path(self):
    """The command as the SConscript writes it, run: jetlink found through the
    path it sets, over the checkout, as on the prebuilt runner. On tinygrad's
    CPU device, since this runs off the comma."""
    import jetlink
    checkout = Path(jetlink.__file__).resolve().parents[1]
    # the path the build has, less anywhere this runner finds jetlink
    base = os.pathsep.join(p for p in sys.path if p and not (Path(p) / 'jetlink').exists())
    with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as out:
      commands = run_sconscript(CAMERAS[:1], jetlink=checkout, pythonpath=base, root=root)
      [(cmd, env)] = [command for target, command in commands.items() if '_lossless_' not in target]
      pkl = Path(out) / 'warp.pkl'
      cmd = cmd.removeprefix('-').rsplit(' --output ', 1)[0] + f' --output {pkl}'
      done = subprocess.run(cmd.replace('python3', sys.executable, 1), shell=True, capture_output=True, text=True,
                            cwd=out, env={**os.environ, **env, 'DEV': 'CPU'}, timeout=600)
      self.assertEqual(done.returncode, 0, done.stderr)
      self.assertTrue(pkl.is_file())


class TestTheLosslessWarp(OpenpilotTestCase):
  def test_a_jetlink_with_lossless_frames_gets_its_warp_beside_the_plain_one(self):
    import jetlink
    from jetlink.openpilot.warp import lossless_path
    checkout = Path(jetlink.__file__).resolve().parents[1]
    w, h = CAMERAS[0]
    plain = str(jetlink_adapter.Adapter().warp_path(w, h, *MODEL))
    lossless = str(lossless_path(plain))
    targets = run_sconscript(CAMERAS[:1], jetlink=checkout)
    self.assertEqual(set(targets), {plain, lossless})
    self.assertTrue(targets[lossless][0].endswith(f'--output {lossless} --lossless'))
    self.assertNotIn('--lossless', targets[plain][0])


class TestOnlyTheBuildCompiles(OpenpilotTestCase):
  def test_only_make_warp_imports_the_compiler(self):
    """compile_modeld is for the build. Imported anywhere else in the adapter it
    would bring the ~9 s compile back to modeld or a provisioning run, where it
    was lost to ignition."""
    tree = ast.parse(Path(jetlink_adapter.__file__).read_text())
    importers = set()
    for fn in ast.walk(tree):
      if isinstance(fn, ast.FunctionDef):
        for node in ast.walk(fn):
          if isinstance(node, ast.ImportFrom) and (node.module or '').endswith('compile_modeld'):
            importers.add(fn.name)
    top = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    self.assertFalse([n for n in top if 'compile_modeld' in ast.dump(n)])
    self.assertEqual(importers, {'make_warp'})


if __name__ == "__main__":
  unittest.main()
