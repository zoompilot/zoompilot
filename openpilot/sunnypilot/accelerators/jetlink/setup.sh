#!/usr/bin/env bash
# Presents the comma as a USB gadget so a Jetson can enumerate it, and links
# the standalone jetlink package onto PYTHONPATH.
#
# The package may live outside the repo until it is a submodule: the updater's
# reset --hard + clean deletes untracked files inside the repo, which would
# take it with it.
set -u
BASEDIR="$(cd "$(dirname "$0")/../../../.." && pwd)"
STATUS=/dev/shm/jetlink-gadget

REPO=""
for candidate in "$BASEDIR/jetlink_repo" /data/jetlink_repo; do
  [ -d "$candidate" ] && { REPO="$candidate"; break; }
done

if [ -z "$REPO" ]; then
  # Most likely a fresh switch to this branch with the submodule never fetched.
  # Say so where the offroad alert can find it rather than leaving the feature
  # mysteriously absent.
  echo "error: the jetlink package is not installed; expected jetlink_repo/ or /data/jetlink_repo" \
    > "$STATUS" 2>/dev/null || true
  exit 0
fi

ln -sfn "$REPO/jetlink" "$BASEDIR/jetlink"
[ -f /AGNOS ] || exit 0

# The endpoints have to exist before jetlinkd or modeld can open them, and
# configuring a gadget needs root while the launcher runs as comma. Do not
# silence a failure: setup_gadget.sh leaves the reason in $STATUS, which is what
# the offroad alert reads.
sudo -n bash "$REPO/scripts/setup_gadget.sh" >/dev/null ||
  echo "jetlink: USB gadget setup failed" >&2
exit 0
