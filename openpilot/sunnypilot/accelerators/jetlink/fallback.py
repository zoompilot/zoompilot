"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from tinygrad import Tensor, TinyJit


def prepare_reset(model):
  """Capture reset before driving, preserving the small JIT's buffer identities.

  Its recurrent state is frozen while the Jetson runs. Returning to those old
  images/features is not a current observation, so fallback starts with the
  same zero history as modeld startup. No queue allocation or JIT compilation
  is deferred to the failure frame.
  """
  queues = tuple(model.input_queues[k] for k in ('img_q', 'big_img_q', 'feat_q', 'desire_q'))

  @TinyJit
  def clear():
    Tensor.realize(*(q.assign(0) for q in queues))

  for _ in range(3):
    clear()

  def reset():
    clear()
    model.prev_desire.fill(0)
    model.npy['prev_feat'].fill(0)
    model.npy['desire'].fill(0)

  return reset
