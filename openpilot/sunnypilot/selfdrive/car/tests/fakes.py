"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Test doubles shared by the sunnypilot overlay tests.
"""


class FakeParams:
  """Dict-backed Params: bools and typed values, records clear_all flags."""

  def __init__(self, **values):
    self.values = dict(values)
    self.cleared: list = []

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value, **kwargs):
    self.values[key] = value

  def get(self, key, **kwargs):
    return self.values.get(key)

  def put(self, key, value, **kwargs):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)

  def clear_all(self, flag):
    self.cleared.append(flag)


def answer(params, outcome, request_id=1):
  """card's hand-back result for a request id, as the consumer gates read it."""
  params.put("StockEcuHandBackResult", {"id": request_id, "outcome": str(outcome)})


class FakeClock:
  """Injectable monotonic clock: set .t directly."""

  def __init__(self):
    self.t = 0.0

  def __call__(self):
    return self.t
