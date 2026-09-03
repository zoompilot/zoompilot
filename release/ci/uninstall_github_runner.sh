#!/usr/bin/env bash
set -euo pipefail

# Optional: --token <removal token> deregisters the runner from GitHub too.
# Without it only the local install is removed (config.sh remove prompts
# interactively when no token is given, which hangs a headless run) and the
# runner entry is left to expire server-side.
GITHUB_TOKEN=""
if [[ "${1:-}" == "--token" ]]; then
    GITHUB_TOKEN="${2:-}"
fi

# Determine BASE_DIR based on mount point
if mountpoint -q /data/media; then
    GITHUB_BASE_DIR="/data/media/0/github"
else
    GITHUB_BASE_DIR="/data/github"
fi

# Define directories and user
BIN_DIR="$GITHUB_BASE_DIR/bin"
BUILDS_DIR="$GITHUB_BASE_DIR/builds"
OPENPILOT_DIR="$GITHUB_BASE_DIR/openpilot"
LOGS_DIR="$GITHUB_BASE_DIR/logs"
CACHE_DIR="$GITHUB_BASE_DIR/cache"
RUNNER_DIR="$GITHUB_BASE_DIR/runner"
RUNNER_USERNAME="github-runner"
USER_GROUPS="comma,gpu,gpio,sudo"

# Function to stop and disable the systemd service
stop_and_uninstall_service() {
    [ -d "$RUNNER_DIR" ] || return 0
    cd "$RUNNER_DIR"
    sudo ./svc.sh stop || true
    sudo ./svc.sh uninstall || true
}

# Function to deregister the runner and remove its registration file
remove_runner() {
    [ -d "$RUNNER_DIR" ] || return 0
    cd "$RUNNER_DIR"
    # Deregister before deleting .runner because config.sh reads it during removal.
    if [ -n "$GITHUB_TOKEN" ] && [ -f .runner ]; then
        sudo su -c "./config.sh remove --token $GITHUB_TOKEN" "$RUNNER_USERNAME" || true
    fi
    sudo rm -f .runner
}

# Function to delete the Github Runner directories
delete_directories() {
    sudo rm -rf "$BIN_DIR/github-runner"
    sudo rm -rf "$GITHUB_BASE_DIR" "$BIN_DIR" "$BUILDS_DIR" "$LOGS_DIR" "$CACHE_DIR" "$OPENPILOT_DIR"
}

# Function to remove the Github Runner user
delete_user() {
    id "$RUNNER_USERNAME" &>/dev/null || return 0
    for group in ${USER_GROUPS//,/ }
    do
       sudo gpasswd -d "$RUNNER_USERNAME" "$group" || true
    done
    sudo userdel -r "$RUNNER_USERNAME" || true
}

# Function to remove sudoers entry
remove_sudoers_entry() {
    sudo sed -i.bak "/${RUNNER_USERNAME} ALL=(ALL) NOPASSWD: ALL/d" /etc/sudoers
}

# Make filesystem writable. The comma is load-bearing: "remount rw /" parses
# "rw" as the device and silently leaves the AGNOS rootfs read-only.
sudo mount -o remount,rw /

# Ensure filesystem is remounted as read-only on script exit
trap "sudo mount -o remount,ro / || true" EXIT

# Call functions
stop_and_uninstall_service
remove_runner
cd /  # leave the runner dir before deleting it
delete_directories
delete_user
remove_sudoers_entry
# End of uninstall script
