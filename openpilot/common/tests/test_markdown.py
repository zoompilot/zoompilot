import os

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.basedir import BASEDIR
from openpilot.common.markdown import parse_markdown


class TestMarkdown(OpenpilotTestCase):
  def test_all_release_notes(self):
    with open(os.path.join(BASEDIR, "CHANGELOG.md")) as f:
      release_notes = f.read().split("\n\n")
      assert len(release_notes) > 5  # fork: our changelog is 7 releases, restore upstream's 10 once it outgrows it

      for rn in release_notes:
        md = parse_markdown(rn)
        assert len(md) > 0
