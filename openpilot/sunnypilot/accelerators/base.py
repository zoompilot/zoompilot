"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The interface an accelerator backend implements.

An accelerator is whatever executes the large driving model: comma's chestnut
board, an attached Jetson over jetlink, anything else a fork adds. Core
openpilot asks the questions below and never names a backend.

Distinct from two axes that already exist and mean other things:
  source  ("qcom" | "chestnut")  which model catalog - sunnypilot/models
  runner  (stock | tinygrad)     which inference stack - ModelManagerSP.Runner

The wire format keeps comma's names: deviceState.chestnutPresent and the
chestnutState message. Renaming cereal fields would break log compatibility for
a cosmetic win, so on the wire "chestnut" means "the active accelerator".
"""
from __future__ import annotations

from collections.abc import Callable
from typing import NamedTuple, Protocol


class Daemon(NamedTuple):
  """An offroad process a backend needs.

  A description rather than a PythonProcess so backends do not import manager,
  which imports them. process_config builds the real process and owns the
  onroad/offroad gating.
  """
  name: str
  module: str
  should_run: Callable[..., bool]


class Accelerator(Protocol):
  """One way of running the large model.

  Every method must be safe to call on any device: a backend whose hardware is
  absent answers False, it does not raise. Callers go through the module-level
  helpers in __init__, which guard anyway, but a backend that throws on a
  machine it does not belong to will spam the log.
  """

  name: str

  def present(self) -> bool:
    """Is the hardware attached?

    Drives deviceState.chestnutPresent, which model catalog the manager
    downloads into, and whether the UI offers large models at all.
    """

  def ready(self) -> bool:
    """Can it run the large model right now?

    Cheap and side-effect free: modeld calls this before it has a model, so no
    link IO and no large file reads. Anything expensive belongs in the daemon.
    """

  def unavailable_reason(self) -> str | None:
    """Why a backend the user asked for cannot run, for the offroad alert.

    None when there is nothing worth interrupting them about, which includes
    hardware simply not being fitted.
    """

  def prepare(self) -> None:
    """Process-wide setup modeld must do before loading. modeld only."""

  def make_model_state(self, cam_w: int, cam_h: int, small=None):
    """The large-model ModelState. Raising falls modeld back to the small model.

    `small` is the small-model ModelState modeld has already loaded on the
    main thread. A backend that only needs the warp borrows it rather than
    loading a second copy of the same pkl on the same device from a thread the
    main thread is racing.
    """

  def make_health_publisher(self, pm, model):
    """Publisher for the chestnutState message, or None if there is no telemetry."""

  def catalog(self) -> str | None:
    """The model-manager catalog this backend's models come from, or None.

    comma's board runs the bundles the model manager downloads into its
    "chestnut" slot. A backend with its own registry answers None, and the
    manager then keeps offering the small-model catalog: a bundle it cannot
    run must not be selectable.
    """

  def daemon(self) -> Daemon | None:
    """An offroad process this backend needs, or None."""
