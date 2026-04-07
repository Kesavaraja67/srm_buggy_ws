#!/bin/bash
# run_arrow_teleop.sh
# Wrapper called by xterm so it gets a proper shell environment.
# xterm cannot execute Python scripts directly (they need an interpreter).
source /opt/ros/humble/setup.bash
source ~/workspaces/srm_buggy_ws/install/setup.bash
exec python3 ~/workspaces/srm_buggy_ws/src/buggy_brain/buggy_brain/arrow_teleop.py "$@"
