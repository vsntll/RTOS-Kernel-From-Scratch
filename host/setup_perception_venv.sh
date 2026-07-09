#!/usr/bin/env bash
# Phase 15 (perception layer): one-time setup for host/perception_node.py's
# Python dependencies (torch, torchvision, ultralytics -- none of which are
# ROS2 or apt packages).
#
# Why a venv instead of `pip install --user`: this WSL image's system
# Python is "externally managed" (PEP 668) and refuses a plain
# `pip install --user torch` outright (confirmed by testing, not assumed --
# it fails immediately with "error: externally-managed-environment").
# `--break-system-packages` would work but installs into the
# system-managed Python's own user site-packages; a venv keeps this
# fully isolated instead.
#
# Why --system-site-packages specifically: rclpy/cv_bridge/sensor_msgs/
# vision_msgs live under /opt/ros/*/... and are put on PYTHONPATH by
# `source /opt/ros/*/setup.bash`, not installed into the system Python's
# own site-packages directory -- a plain venv (without this flag) can't
# see them even with ROS2 sourced, since venvs don't automatically forward
# PYTHONPATH-based additions the same way. --system-site-packages plus
# sourcing ROS2's setup.bash (in either order -- both are just PYTHONPATH
# entries) makes both visible together, confirmed live: `import rclpy` and
# `import cv_bridge` succeed with this specific combination and fail
# without --system-site-packages, or without ROS2 sourced, or both.
#
# Why numpy is pinned below 2.0 in this venv specifically: torch (a
# dependency of ultralytics) wants numpy>=2 by its own dependency
# resolution, but the *system's* matplotlib (pulled in transitively by
# ultralytics importing its own training-only modules even for pure
# inference -- an ultralytics packaging quirk, not something this project
# controls) was compiled against numpy 1.x's ABI and crashes on import
# under numpy 2.x. Both torch/torchvision (matched-version pair) and the
# system matplotlib import cleanly together specifically at numpy<2 --
# found by testing both directions of this conflict, not guessed.
#
# Requires apt's python3.12-venv package (or your distro's equivalent)
# already installed -- if `python3 -m venv` fails with "ensurepip is not
# available", that's the missing piece: `sudo apt install python3.12-venv`.
#
# Usage:
#   bash host/setup_perception_venv.sh
#   # afterwards, every time you want to run the perception node:
#   source /opt/ros/*/setup.bash
#   source ~/perception_venv/bin/activate
#   python3 host/perception_node.py
set -euo pipefail

VENV_DIR="${1:-$HOME/perception_venv}"

if [ -d "$VENV_DIR" ]; then
    echo "warning: $VENV_DIR already exists -- delete it first for a clean rebuild" >&2
fi

python3 -m venv --system-site-packages "$VENV_DIR"
source "$VENV_DIR/bin/activate"

pip install --upgrade pip -q
# Matched pair, one install -- installing torch and torchvision
# separately produced a real version mismatch (torchvision's compiled
# `nms` op failed to register against a differently-versioned torch),
# found by testing, fixed by installing both together instead of
# guessing a compatible pair of version numbers by hand.
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics
pip install "numpy<2"

echo ""
echo "done. verify with:"
echo "  source /opt/ros/*/setup.bash && source $VENV_DIR/bin/activate"
echo "  python3 -c 'import rclpy, cv_bridge, torch, ultralytics; print(\"all OK\")'"
