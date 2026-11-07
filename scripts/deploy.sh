#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
robot="${1:-wheeltec@192.168.10.101}"
ssh_opts=(-o ControlMaster=auto -o ControlPersist=3600 -o ControlPath=/tmp/wheeltec-slam-ssh-%C)
ssh "${ssh_opts[@]}" "$robot" 'mkdir -p ~/wheeltec_slam_ws/src/wheeltec_rtabmap_lab ~/wheeltec_slam_runtime/maps ~/wheeltec_slam_runtime/logs'
tar -czf - package.xml CMakeLists.txt launch config scripts docs | ssh "${ssh_opts[@]}" "$robot" 'tar xzf - -C ~/wheeltec_slam_ws/src/wheeltec_rtabmap_lab'
ssh "${ssh_opts[@]}" "$robot" bash -s <<'REMOTE'
set -e
source /opt/ros/noetic/setup.bash
cd ~/wheeltec_slam_ws
chmod +x src/wheeltec_rtabmap_lab/scripts/*.py
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release > ~/wheeltec_slam_runtime/logs/build.log 2>&1
REMOTE
