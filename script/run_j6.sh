#!/bin/bash
# DrEngine 车端(J6)原生运行脚本。source ROS humble + /app/road/m22_road/setup.bash + 匹配驱动的 RMW。
# 位于 <工程根>/script/run_j6.sh，工程根 = 上一级。
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NODE="$DIR/build/dr_engine_node"
PARAMS="${PARAMS:-$DIR/config/dr_params.yaml}"
set +u
source /opt/ros/humble/setup.bash
source /app/road/m22_road/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # 与 m22 驱动一致（裸 ros2 CLI 默认是 rmw_communication）
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
set -u
mkdir -p "$DIR/run" "$DIR/log"
export ROS_DISABLE_LOANED_MESSAGES=1
exec "$NODE" --ros-args --params-file "$PARAMS" "$@"
