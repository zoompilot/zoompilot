import subprocess

from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.models.compile_ref import COMPILE_SCRIPTS, get_compile_ref, compile_ref_from_blob_shas


class TestCompileRef(OpenpilotTestCase):
  def test_matches_git_blob_shas(self):
    # the prebuilt workflow builds the same ref from the GitHub contents API blob shas
    shas = subprocess.check_output(["git", "hash-object", *COMPILE_SCRIPTS], cwd=BASEDIR, text=True).split()
    assert get_compile_ref() == compile_ref_from_blob_shas(shas)
