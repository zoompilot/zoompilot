"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Run openpilot's large (chestnut) driving models on an attached Jetson.

The transport, protocol and TensorRT server live in the standalone `jetlink`
package so other forks can reuse them. Everything in this directory is the
openpilot-specific glue and nothing else.
"""
