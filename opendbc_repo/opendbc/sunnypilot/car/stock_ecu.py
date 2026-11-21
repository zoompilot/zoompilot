"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The stock ECU transition contract.

A brand that silences a stock ECU for openpilot longitudinal (Mazda's radar) reports where
that ownership stands through one state, so card and the UI never read brand state. The
controller exposes it as `stock_ecu_state`; a controller without the attribute silences
nothing (NOT_NEEDED).

The state is the driver's view: what, if anything, they have to do. Ownership details (which
prerequisite, which UDS reply) go to carlog. READY is the vehicle's own guard on the silence
(the two-master guard), the point from which normal
engagement can follow; a session acknowledgement is never readiness. RESTORED and FAILED are
also the answers the lifecycle's hand-back request gets (stock_ecu_handback.py).
"""
from enum import StrEnum


class StockEcuState(StrEnum):
  NOT_NEEDED = "notNeeded"           # nothing to take over on this platform or in this mode
  STARTING = "starting"              # prerequisites, request or silence guard still pending: wait
  PARK_TO_TAKE_OVER = "parkToTakeOver"  # this session takes over at the next stop
  STOCK_CRUISE_ON = "stockCruiseOn"  # the driver's own stock engagement holds the takeover
  READY = "ready"                    # owned and guarded; engage normally
  RESTORING = "restoring"            # the default session is requested, stock traffic not back
  RESTORED = "restored"              # sustained stock traffic after the ordered hand-back
  FAILED = "failed"                  # a bounded attempt ended without the ECU answering


# States in which a SET/RES press engages as on a stock car: nothing to explain to the driver.
ENGAGES_NORMALLY = frozenset({StockEcuState.NOT_NEEDED, StockEcuState.READY, StockEcuState.RESTORED})
