#!/usr/bin/env bash
set -eo pipefail
source /home/wheeltec/wheeltec_robot/devel/setup.bash
source /home/wheeltec/wheeltec_slam_ws/devel/setup.bash
export CMAKE_PREFIX_PATH="/home/wheeltec/wheeltec_slam_ws/devel:/home/wheeltec/wheeltec_robot/devel:/opt/ros/noetic"
export LD_LIBRARY_PATH="/home/wheeltec/wheeltec_slam_ws/devel/lib:/home/wheeltec/wheeltec_robot/devel/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
vendor_core=/home/wheeltec/wheeltec_slam_runtime/vendor/rtabmap-0.21.13/opt/ros/noetic/lib/aarch64-linux-gnu
if [[ -d "$vendor_core" ]]; then export LD_LIBRARY_PATH="$vendor_core:/home/wheeltec/wheeltec_slam_runtime/vendor/rtabmap-0.21.13/opt/ros/noetic/lib:$LD_LIBRARY_PATH"; fi
# Recover vendor package discovery without modifying existing workspace files.
export ROS_PACKAGE_PATH="/home/wheeltec/wheeltec_slam_ws/src:/home/wheeltec/wheeltec_robot/src:/opt/ros/noetic/share"
vendor_prefix=/home/wheeltec/wheeltec_slam_runtime/vendor/rtabmap-0.21.13/opt/ros/noetic
if [[ -d "$vendor_prefix/share/rtabmap_slam" ]]; then
 export CMAKE_PREFIX_PATH="$vendor_prefix:$CMAKE_PREFIX_PATH"
 export ROS_PACKAGE_PATH="$vendor_prefix/share:$ROS_PACKAGE_PATH"
fi
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
runtime=/home/wheeltec/wheeltec_slam_runtime
mkdir -p "$runtime/logs" "$runtime/maps"
case "${1:-}" in
 master) exec roscore ;;
 recover)
  database=$(python3 -c 'import json,pathlib; p=pathlib.Path.home()/"wheeltec_slam_runtime/maps/session_state.json"; print(json.loads(p.read_text())["active_path"] if p.exists() else str(p.parent/"lab.db"))')
  localization=true
  if [[ ! -f "$database" ]]; then localization=false; fi
  exec roslaunch wheeltec_rtabmap_lab mapping.launch localization:="$localization" database:="$database" ;;
 camera) exec roslaunch wheeltec_rtabmap_lab camera.launch ;;
 sensors) exec roslaunch wheeltec_rtabmap_lab sensors.launch motion_enabled:="${2:-false}" wheelbase:=0.143 ;;
 mapping) exec roslaunch wheeltec_rtabmap_lab mapping.launch database:="${2:-$runtime/maps/lab.db}" ;;
 localization) exec roslaunch wheeltec_rtabmap_lab mapping.launch localization:=true database:="${2:-$runtime/maps/lab.db}" ;;
 *) echo 'Usage: start_vehicle.sh camera|sensors [true|false]|mapping [database]|localization [database]' >&2; exit 2 ;;
esac
