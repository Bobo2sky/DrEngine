#!/usr/bin/env bash
# ============================================================================
#  DrEngine 交叉编译：为 J6 域控（aarch64 / Debian 12 / ROS 2 humble）产出 dr_engine_node
#
#  用法:
#    ./script/build_arm.sh              # 编译（依赖已就绪时最快路径）
#    ./script/build_arm.sh --msgs       # 先交叉编译消息包（chassis_msgs 等），再编 dr_engine_node
#    ./script/build_arm.sh --shell      # 进容器手动排查
#
#  前置（与 hg-slam 的 build_arm.sh 共用同一套，缺哪项脚本会明确报出来）：
#    1) docker 镜像 j6_debian_12:ros2-dev-v6
#    2) ${J6_ROOT}/j6_third_party  公共依赖仓（含 cmake/j6-toolchain.cmake），只读挂 /opt/third_party
#    3) ${J6_ROOT}/ipc_msgs/src    消息仓源码（--msgs 需要），挂 /ipc_src
#    4) aarch64 消息覆盖层（chassis_msgs 的 .so/include）—— 由 --msgs 产出或手工放到
#       ${J6_ROOT}/output/arm_ws/install ，容器内挂到 /arm_msgs
#
#  容器内挂载:
#    ${仓库}/src/localization/DrEngine  -> /src/module/src/localization/DrEngine  (rw, 产物留 build-arm/)
#    ${J6_ROOT}/output/arm_ws/install -> /arm_msgs                              (ro, aarch64 消息覆盖层)
#
#  产物: src/localization/DrEngine/build-arm/dr_engine_node (aarch64)
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"          # .../DrEngine
ROOT_DIR="$(cd "${PROJECT_DIR}/../../.." && pwd)"      # 仓库根
J6_ROOT="${J6_ROOT:-${HOME}/j6-dev}"
IMAGE="${IMAGE:-j6_debian_12:ros2-dev-v6}"
JOBS="${JOBS:-6}"
THIRD_PARTY="${J6_ROOT}/j6_third_party"
# aarch64 消息覆盖层：默认用 hg-slam 交叉编译已有的 ipc_msgs 产物（含 chassis_msgs）
ARM_MSGS="${ARM_MSGS:-${J6_ROOT}/output/install/ipc_msgs}"
# 车端运行时消息目录（写进产物 RPATH；车端原生编译时 CMake 默认值也是它）
ARM_RUNTIME_MSGS="${ARM_RUNTIME_MSGS:-/data/runtime/ros_msg/install}"
[ -d "$ARM_MSGS/chassis_msgs" ] || ARM_MSGS_DEFAULT2="${J6_ROOT}/output/arm_ws/install"
TOOLCHAIN_IN_CONTAINER=/opt/third_party/cmake/j6-toolchain.cmake
ROOT_IN_CONTAINER=/src/module

MODE=node
for arg in "$@"; do
  case "$arg" in
    --msgs)  MODE=msgs ;;
    --node)  MODE=node ;;
    --shell) MODE=shell ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg（见 --help）" >&2; exit 2 ;;
  esac
done

die() { echo "[ERR] $*" >&2; exit 1; }

# ---- 前置检查 ----
command -v docker > /dev/null || die "未找到 docker"
docker image inspect "$IMAGE" > /dev/null 2>&1 || die "缺少 docker 镜像 $IMAGE"
[ -d "$THIRD_PARTY" ] || die "缺少 $THIRD_PARTY（公共依赖仓）"
[ -f "$THIRD_PARTY/cmake/j6-toolchain.cmake" ] || die "缺少 $THIRD_PARTY/cmake/j6-toolchain.cmake"
if [ "$MODE" = "node" ] && [ ! -d "$ARM_MSGS/chassis_msgs" ]; then
    die "缺少 aarch64 消息覆盖层: $ARM_MSGS/chassis_msgs
  先跑: ./script/build_arm.sh --msgs（或把已交叉编译好的 chassis_msgs 等放到该目录）"
fi

# ---- 容器内执行脚本 ----
CONTAINER_SH=$(cat <<'EOS'
set -e
# 容器自带的交叉编译环境（PATH/CMAKE_PREFIX_PATH/PYTHONPATH/AMENT_PREFIX_PATH 全在这里配）
source /cross/env.sh
cd /src/module/src/localization/DrEngine
echo "== 容器内环境 =="
grep -m1 . /etc/os-release || true
ls /opt/ros || true
case "__MODE__" in
  msgs)
    echo "[msgs] 交叉编译消息包 -> /arm_msgs"
    mkdir -p /arm_msgs
    # 说明: chassis_msgs 等消息包的交叉编译沿用 hg-slam 的流程（catkin/cmake 交叉编译 ipc_msgs），
    #       产物 install 目录即 /arm_msgs。此处若已存在（-v 挂载）直接复用。
    if [ -d /arm_msgs/chassis_msgs ]; then
      echo "[msgs] 复用已存在的 /arm_msgs/chassis_msgs"; exit 0
    fi
    if [ -d /ipc_src ]; then
      echo "[msgs] 检测到 /ipc_src，请参考 hg-slam/script/arm_build_container.sh 的 ipc 段完成交叉编译"; exit 3
    fi
    echo "[msgs] 既无 /arm_msgs 也无 /ipc_src，无法自动产出消息覆盖层"; exit 4
    ;;
  node)
    echo "[node] 配置 + 编译 dr_engine_node (aarch64)"
    BUILD=/work/build/imudr-arm
    rm -rf "$BUILD" && mkdir -p "$BUILD"
    if ! cmake -S . -B "$BUILD" \
            -DCMAKE_TOOLCHAIN_FILE=__TOOLCHAIN__ \
            -DCMAKE_BUILD_TYPE=Release \
            -DMSG_DIR=/arm_msgs -DIMUDR_DEPLOY_MSGDIR=${ARM_RUNTIME_MSGS:-/data/runtime/ros_msg/install} > /tmp/arm-cfg.log 2>&1; then
        echo "---- cmake 配置失败，日志尾部 ----"; tail -40 /tmp/arm-cfg.log; exit 1
    fi
    grep -E "DrEngine:|Detected|RPATH" /tmp/arm-cfg.log || true
    cmake --build "$BUILD" -j__JOBS__ 2>&1 | tail -30
    # 产物拷回宿主机工程目录（/work 是容器工作区，产物留 build-arm/ 便于部署）
    mkdir -p build-arm && cp -f "$BUILD/dr_engine_node" build-arm/dr_engine_node
    file build-arm/dr_engine_node
    ;;
  shell) echo "[shell] 进入交互"; exec bash ;;
esac
EOS
)
CONTAINER_SH="${CONTAINER_SH//__MODE__/$MODE}"
CONTAINER_SH="${CONTAINER_SH//__TOOLCHAIN__/$TOOLCHAIN_IN_CONTAINER}"
CONTAINER_SH="${CONTAINER_SH//__JOBS__/$JOBS}"

echo "== DrEngine ARM 交叉编译 =="
echo "  镜像      : $IMAGE"
echo "  仓库根    : $ROOT_DIR"
echo "  J6_ROOT   : $J6_ROOT"
echo "  ARM 消息层: $ARM_MSGS   (编译期挂载 /arm_msgs)"
echo "  车端运行时: $ARM_RUNTIME_MSGS   (写入 RPATH)"
echo "  模式      : $MODE"
echo

TTY_FLAG=""
[ -t 0 ] && [ -t 1 ] && TTY_FLAG="-it"   # 非交互（CI/管道）时不能加 -it，否则 docker 报 "input device is not a TTY"

docker run --rm $TTY_FLAG --user "$(id -u):$(id -g)" \
  --name dr_engine_arm_$$ \
  -v "$ROOT_DIR":$ROOT_IN_CONTAINER \
  -v "$THIRD_PARTY":/opt/third_party:ro \
  -v "$ARM_MSGS":/arm_msgs:ro \
  -v "$J6_ROOT/output":/work \
  -v "$J6_ROOT/ipc_msgs/src":/ipc_src:ro \
  -w /src/module \
  "$IMAGE" \
  bash -lc "$CONTAINER_SH"

echo
echo "== 完成 =="
ls -l "$PROJECT_DIR/build-arm/dr_engine_node" 2>/dev/null || true
