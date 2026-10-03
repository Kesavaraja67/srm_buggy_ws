#!/bin/bash
# =============================================================================
# SRM Autonomous Electric Campus Buggy — Demo Launch Script
# Usage: bash run_demo.sh [--headless] [--world <world_name>]
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

# Default options
WORLD="srm_campus"
HEADLESS=false

# Parse args
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --headless) HEADLESS=true ;;
        --world) WORLD="$2"; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

echo "========================================"
echo " SRM Autonomous Buggy — Nav2 Demo Runner"
echo " World : $WORLD"
echo " Headless: $HEADLESS"
echo " Lidar Map: my_campus_map.pgm"
echo "========================================"

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace install (after build)
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[WARN] Workspace not built yet. Run 'colcon build' first."
    echo "       cd $WS_DIR && colcon build --symlink-install"
    exit 1
fi

# Kill any lingering processes from previous runs
echo "[INFO] Cleaning up old processes..."
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f path_planner 2>/dev/null || true
pkill -f nav2_destination_sender 2>/dev/null || true
pkill -f waypoint_follower 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
sleep 1

# Launch the official Nav2 stack configuration
echo "[INFO] Starting Gazebo + Nav2 + AMCL with LiDAR Map..."
if [ "$HEADLESS" = true ]; then
    export HEADLESS=1
    ros2 launch buggy_bringup nav2_launch.py world:="$WORLD" gui:=false
else
    ros2 launch buggy_bringup nav2_launch.py world:="$WORLD" gui:=true
fi
