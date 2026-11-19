#!/bin/bash
# ============================================================================
#  DrEngine 一键编译（独立 CMake，不进入 hg-slam 的构建体系）
#  依赖: ROS 2 + 消息层 m22_road_msgs（本机: /workspace/J6/vehiche/install）+ Eigen3
# ============================================================================
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"     # .../DrEngine
ROOT="$(cd "$DIR/../../.." && pwd)"                        # 仓库根 (/home/wb/hg-slam)
JOBS="${JOBS:-8}"

# ROS 2 的 setup.bash 里有未加保护的变量引用，set -u 下会致命退出 ⇒ 期间临时关掉
set +u
# ROS：显式 > $ROS_DISTRO > 自动扫描（车端 humble / 开发机 jazzy）
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    for cand in /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/*/setup.bash; do
        [ -f "$cand" ] && { source "$cand"; break; }
    done
fi
# 消息层（m22_road_msgs）：开发机 vehiche install 或 车端运行时路径
for cand in \
    "${COMMON_SETUP:-}" \
    "/workspace/J6/vehiche/install/setup.bash" \
    "$ROOT/app/common/install/setup.bash" \
    "/data/runtime/ros_msg/install/local_setup.bash" \
    "/data/runtime/ros_msg/install/setup.bash"; do
    [ -n "$cand" ] && [ -f "$cand" ] && { source "$cand"; echo "  消息层: $cand"; break; }
done
set -u

echo "== DrEngine build =="
echo "  源码   : $DIR"
echo "  仓库根 : $ROOT"
echo "  ROS    : ${ROS_DISTRO:-未加载}"
echo "  架构   : $(uname -m)   （消息层默认取自 source 的 install，如 /workspace/J6/vehiche/install）"

cmake -S "$DIR" -B "$DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$DIR/build" -j"$JOBS"

echo
echo "== 完成: $DIR/build/dr_engine_node =="
echo "运行:"
echo "  $DIR/script/run.sh                                    # 只起节点"
echo "  BAG=<bag>.mcap RVIZ=1 $DIR/script/run.sh              # 回放+节点+rviz 一键"
echo "  (脚本会自动 source ROS 与消息层 install（含 m22_road_msgs），避免缺 libm22_road_msgs 的报错)"
