#!/bin/bash
# ============================================================================
#  DrEngine 运行脚本（后台常驻）—— 自动准备环境（ROS 2 + 消息层 m22_road_msgs）后
#  把 dr_engine_node 放到后台运行，脚本立即返回。
#
#  用法:
#    ./run.sh start                     # 后台启动（参数全部取自 config/dr_params.yaml）
#    ./run.sh stop                      # 停止（先 SIGINT 优雅退出，超时再 SIGKILL），连带收掉回放/rviz
#    ./run.sh restart                   # 重启
#    ./run.sh status                    # 查看运行状态（PID/存活/最近一行日志）
#    ./run.sh logs [-n 50] [-f]         # 查看日志（默认 log/dr.text，-f 持续跟踪）
#    ./run.sh check                     # 现场自检：进程状态 + 各话题发布频率 + 最近日志
#    ./run.sh start -p publish_rate_hz:=20.0   # 临时覆盖某个参数（不写进配置文件）
#
#  说明: use_sim_time 统一在【配置文件 config/dr_params.yaml】里设置，本脚本不再传它
#        （回放用 true，实车改 false）。
#
#  环境变量（都在 start 时生效）:
#    PARAMS=other.yaml   参数文件（默认 config/dr_params.yaml）
#    BAG=/path/bag目录   顺便起回放（--clock，只放参数文件里的 imu_topic/speed_topic），再起节点
#    RVIZ=1              顺便起 rviz2（Fixed Frame=odom）
#
#  运行时文件: run/{node,player,rviz}.pid  run/node.out(节点 stdout/stderr)
#  节点日志  : log/dr.text（节点自己写，追加）
# ============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"     # .../DrEngine
ROOT="$(cd "$DIR/../../.." && pwd)"                        # 仓库根 (/home/wb/hg-slam)
NODE="$DIR/build/dr_engine_node"
PARAMS="${PARAMS:-$DIR/config/dr_params.yaml}"
ROS_SETUP="${ROS_SETUP:-}"                                     # 可用环境变量覆盖
PLAY_BAG="$ROOT/app/localization/play_bag.sh"
QOS_YAML="$ROOT/app/localization/qos_replay.yaml"
# 消息层候选（按序尝试；都没有则依赖二进制里写死的 RPATH，不报错）：
#   · 开发机 vehiche install（含 m22_road_msgs）: /workspace/J6/vehiche/install
#   · 开发机仓库布局 : <仓库>/app/common/install
#   · 车端(J6)运行时 : /data/runtime/ros_msg/install
#   · 也可用 COMMON_SETUP=/path/setup.bash 显式指定
COMMON_CANDIDATES=(
    "${COMMON_SETUP:-}"
    "/workspace/J6/vehiche/install/setup.bash"
    "$ROOT/app/common/install/setup.bash"
    "/data/runtime/ros_msg/install/local_setup.bash"
    "/data/runtime/ros_msg/install/setup.bash"
)
RUN_DIR="$DIR/run"
LOG_FILE="$DIR/log/dr.text"
NODE_PID="$RUN_DIR/node.pid"
PLAYER_PID="$RUN_DIR/player.pid"
RVIZ_PID="$RUN_DIR/rviz.pid"

green() { printf '\033[0;32m%s\033[0m\n' "$*"; }
info()  { printf '\033[0;36m[INFO]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[WARN]\033[0m %s\n' "$*"; }
err()   { printf '\033[0;31m[ERR ]\033[0m %s\n' "$*" >&2; }

usage() { awk 'NR>1 { if ($0 ~ /^#/) { sub(/^# ?/, ""); print } else { exit } }' "${BASH_SOURCE[0]}"; }

# ---------- 工具 ----------
alive() { [ -n "${1:-}" ] && kill -0 "$1" 2>/dev/null; }

read_pid() { [ -f "$1" ] && cat "$1" 2>/dev/null || true; }

pidfile_alive() {   # pidfile_alive <pidfile> -> echo pid if alive
    local p; p="$(read_pid "$1")"
    if alive "$p"; then echo "$p"; fi
}

prepare_env() {
    [ -x "$NODE" ]   || { err "未找到可执行文件 $NODE（先跑 bash $DIR/script/build.sh）"; exit 1; }
    [ -f "$PARAMS" ] || { err "参数文件不存在: $PARAMS"; exit 1; }
    # ROS 的 setup.bash 有未加保护的变量，set -u 下会致命 ⇒ 期间关掉
    set +u
    # --- ROS 环境：显式指定 > $ROS_DISTRO > 自动扫描 /opt/ros/* ---
    if [ -z "${ROS_SETUP:-}" ] || [ ! -f "${ROS_SETUP:-}" ]; then
        if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
            ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
        else
            # 优先 humble（车端），再 jazzy 等
            for cand in /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/*/setup.bash; do
                [ -f "$cand" ] && { ROS_SETUP="$cand"; break; }
            done
        fi
    fi
    if [ -n "${ROS_SETUP:-}" ] && [ -f "$ROS_SETUP" ]; then
        source "$ROS_SETUP"; info "ROS    : $ROS_SETUP (${ROS_DISTRO:-?})"
    else
        warn "未找到 ROS setup.bash（/opt/ros/*），若 ldd 缺库请检查 ROS 安装"
    fi

    # --- 消息层（m22_road_msgs）：多个候选，缺了也不致命（走 RPATH）---
    OVERLAY=""
    for cand in "${COMMON_CANDIDATES[@]}"; do
        [ -n "$cand" ] && [ -f "$cand" ] && { OVERLAY="$cand"; break; }
    done
    if [ -n "$OVERLAY" ]; then
        source "$OVERLAY"; info "消息层 : $OVERLAY"
    else
        warn "未找到消息层 setup.bash（试过: ${COMMON_CANDIDATES[*]}）"
        warn "  ⇒ 依赖二进制里写死的 RPATH（编译期 -DIMUDR_DEPLOY_MSGDIR）；若下面 ldd 报缺库请补路径"
    fi
    set -u
    local missing
    missing="$(ldd "$NODE" 2>/dev/null | awk '/not found/{print $1}' | sort -u | tr '\n' ' ')"
    if [ -n "${missing// /}" ]; then
        err "节点缺共享库: $missing"
        err "  提示: 找 m22_road_msgs: ls /workspace/J6/vehiche/install/m22_road_msgs/lib 2>/dev/null"
        err "        或 source 消息层后重试: COMMON_SETUP=/workspace/J6/vehiche/install/setup.bash $0 start"
        exit 1
    fi
    mkdir -p "$RUN_DIR" "$DIR/log"
    export ROS_DISABLE_LOANED_MESSAGES=1   # 关掉 "Loaned messages ..." 告警噪音（我们是 const-ref 回调，安全）
}

spawn() {   # spawn <pidfile> <logfile> <命令串>  —— setsid 起独立进程组，便于整组收掉
    local pidfile="$1" logfile="$2"; shift 2
    setsid bash -c "$*" > "$logfile" 2>&1 &
    local pid=$!
    echo "$pid" > "$pidfile"
    echo "$pid"
}

kill_pid() {   # kill_pid <pidfile> <名字> [信号]
    local pidfile="$1" name="$2" sig="${3:-INT}"
    local pid; pid="$(read_pid "$pidfile")"
    if ! alive "$pid"; then rm -f "$pidfile"; return 0; fi
    kill -"$sig" -"$pid" 2>/dev/null || kill -"$sig" "$pid" 2>/dev/null || true
    for _ in $(seq 1 25); do alive "$pid" || break; sleep 0.2; done
    if alive "$pid"; then
        kill -9 -"$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null || true
        warn "$name (PID $pid) 未响应 SIG$sig，已强杀"
    fi
    rm -f "$pidfile"
    return 0
}

# 扫描本机所有同名节点进程（按 /proc/*/exe 精确比对，避免误杀/漏杀）
scan_instances() {
    local t pid out=""
    for e in /proc/[0-9]*/exe; do
        t="$(readlink "$e" 2>/dev/null || true)"
        [ "$t" = "$NODE" ] || continue
        pid="$(basename "$(dirname "$e")")"
        out+="$pid "
    done
    echo "$out"
}

# 从参数文件里取某个键的值（去引号）
cfg_get() {
    [ -f "$PARAMS" ] || return 0
    awk -v k="$1" 'index($0, k ":"){ sub(/.*: */,""); gsub(/"/,""); sub(/#.*/,""); gsub(/[ \t]+$/,""); print; exit }' "$PARAMS"
}

# 启动后健康检查：配置的话题是否真的有发布者
health_check() {
    local imu_t spd_t a
    imu_t="$(cfg_get imu_topic)";   imu_t="${imu_t:-/m22/imu/data_raw}"
    spd_t="$(cfg_get speed_topic)"; spd_t="${spd_t:-/ins/velocity}"
    for a in "$@"; do                     # 命令行 -p imu_topic:=xxx 覆盖参数文件
        case "$a" in
            imu_topic:=*)   imu_t="${a#imu_topic:=}" ;;
            speed_topic:=*) spd_t="${a#speed_topic:=}" ;;
        esac
    done
    local topics; topics="$(ros2 topic list 2>/dev/null || true)"
    echo "$topics" | grep -qx "$imu_t"   || warn "话题 $imu_t 当前没有发布者（bag/驱动没起？或 imu_topic 配错）"
    echo "$topics" | grep -qx "$spd_t"   || warn "话题 $spd_t 当前没有发布者（bag/驱动没起？或 speed_topic 配错）"
}

# ---------- 子命令 ----------
cmd_start() {
    local already; already="$(pidfile_alive "$NODE_PID")"
    if [ -n "$already" ]; then
        warn "节点已在运行 (PID $already)，如需重启请用: ./run.sh restart"
        return 0
    fi
    # 防"多实例同时写 log/dr.text"：本脚本之外还跑着同名节点时拒绝启动
    local others; others="$(scan_instances)"
    if [ -n "${others// /}" ] && [ "${FORCE:-0}" != "1" ]; then
        err "检测到已有 dr_engine_node 在运行: PID $others"
        err "  它们会与本实例同时写 $LOG_FILE，先清理: ./run.sh stop   或  FORCE=1 强制启动"
        exit 1
    fi
    prepare_env

    # 1) 可选：回放（必须先起，节点晚到才拿得到数据）
    if [ -n "${BAG:-}" ]; then
        [ -e "$BAG" ] || { err "bag 不存在: $BAG"; exit 1; }
        # 回放话题从参数文件取（imu_topic + speed_topic），兜底默认值
        local imu_t spd_t play_cmd
        imu_t="$(cfg_get imu_topic)";   imu_t="${imu_t:-/m22/imu/data_raw}"
        spd_t="$(cfg_get speed_topic)"; spd_t="${spd_t:-/ins/velocity}"
        if [ -f "$PLAY_BAG" ]; then
            local pargs=("$BAG" --clock --topics "$imu_t" "$spd_t")
            [ -f "$QOS_YAML" ] && pargs+=(--qos-profile-overrides-path "$QOS_YAML")
            play_cmd="bash '$PLAY_BAG' ${pargs[*]}"
        else
            # 仓库回放脚本不在（非仓库布局）⇒ 直接用 ros2 bag play 兜底
            warn "未找到回放脚本 $PLAY_BAG，改用 ros2 bag play 直接回放"
            play_cmd="ros2 bag play '$BAG' --clock --topics '$imu_t' '$spd_t'"
        fi
        local ppid; ppid="$(spawn "$PLAYER_PID" "$RUN_DIR/player.out" "$play_cmd")"
        echo "  回放    : PID $ppid  ($BAG, 话题: $imu_t $spd_t)"
        sleep 12      # 等 mcap 索引加载 + 起播
    fi

    # 2) 可选：rviz2
    if [ "${RVIZ:-0}" = "1" ]; then
        if command -v rviz2 > /dev/null 2>&1; then
            local simt; simt="$(cfg_get use_sim_time)"; simt="${simt:-false}"
            local rpid; rpid="$(spawn "$RVIZ_PID" "$RUN_DIR/rviz.out" \
                "rviz2 -d '$DIR/rviz/dr_engine.rviz' -p use_sim_time:=$simt")"
            echo "  rviz2   : PID $rpid"
            sleep 3
        else
            warn "未找到 rviz2，跳过"
        fi
    fi

    # 3) 节点（后台；自身还会把日志写进 log/dr.text）
    # 注意: use_sim_time 由参数文件给出，这里不再追加（如需临时覆盖可传 -p use_sim_time:=false）
    local npid; npid="$(spawn "$NODE_PID" "$RUN_DIR/node.out" \
        "'$NODE' --ros-args --params-file '$PARAMS' ${*:-}")"
    sleep 2
    if ! alive "$npid"; then
        err "节点启动失败（PID $npid 已退出），最后 20 行 $RUN_DIR/node.out:"
        tail -20 "$RUN_DIR/node.out" 2>/dev/null || true
        rm -f "$NODE_PID"
        exit 1
    fi

    health_check "$@"
    green "DrEngine 已在后台运行"
    echo "  PID     : $npid   （pidfile: $NODE_PID）"
    echo "  参数    : $PARAMS"
    echo "  仿真钟  : use_sim_time=$(cfg_get use_sim_time)（取自 $PARAMS）"
    echo "  日志    : $LOG_FILE"
    echo "  stdout  : $RUN_DIR/node.out"
    echo
    echo "停止: ./run.sh stop     状态: ./run.sh status     跟踪日志: ./run.sh logs -f"
    return 0
}

cmd_stop() {
    local stopped=0
    if pidfile_alive "$NODE_PID" > /dev/null; then
        kill_pid "$NODE_PID" "dr_engine_node" INT; echo "  已停止节点"; stopped=1
    else
        rm -f "$NODE_PID"; echo "  节点未在运行"
    fi
    if pidfile_alive "$PLAYER_PID" > /dev/null; then
        kill_pid "$PLAYER_PID" "回放" TERM; echo "  已停止回放"; stopped=1
    fi
    if pidfile_alive "$RVIZ_PID" > /dev/null; then
        kill_pid "$RVIZ_PID" "rviz2" TERM; echo "  已停止 rviz2"; stopped=1
    fi
    # 兜底：pidfile 丢失/外部启动的残留实例
    local leftovers; leftovers="$(scan_instances)"
    if [ -n "${leftovers// /}" ]; then
        for pid in $leftovers; do
            kill -INT "$pid" 2>/dev/null || true
            for _ in $(seq 1 25); do alive "$pid" || break; sleep 0.2; done
            alive "$pid" && kill -9 "$pid" 2>/dev/null || true
        done
        echo "  已清理残留实例: $leftovers"; stopped=1
    fi
    [ "$stopped" = "1" ] && green "完成" || green "没有需要停止的进程"
    return 0
}

cmd_status() {
    local npid ppid rpid
    npid="$(pidfile_alive "$NODE_PID")"
    ppid="$(pidfile_alive "$PLAYER_PID")"
    rpid="$(pidfile_alive "$RVIZ_PID")"
    if [ -n "$npid" ]; then
        local up; up="$(ps -o etime= -p "$npid" 2>/dev/null | tr -d ' ')"
        green "节点运行中: PID $npid  已运行 $up"
    else
        warn "节点未运行"
    fi
    [ -n "$rpid" ] && echo "  rviz2 : PID $rpid" || true
    [ -n "$ppid" ] && echo "  回放  : PID $ppid" || true
    echo "  日志  : $LOG_FILE"
    if [ -f "$LOG_FILE" ]; then
        echo "  最近一行:"
        tail -n 20 "$LOG_FILE" | grep -v '^[[:space:]]*$' | tail -1 | cut -c1-200 || true
    fi
    return 0
}

cmd_check() {
    cmd_status
    echo
    echo "---- 话题频率（每项采样约 3s）----"
    local imu_t spd_t
    imu_t="$(cfg_get imu_topic)";   imu_t="${imu_t:-/m22/imu/data_raw}"
    spd_t="$(cfg_get speed_topic)"; spd_t="${spd_t:-/ins/velocity}"
    local topics; topics="$(ros2 topic list 2>/dev/null || true)"
    for t in "$imu_t" "$spd_t" /tf /dr_odom /total_mileage /dr_path; do
        if echo "$topics" | grep -qx "$t"; then
            local hz; hz="$(timeout 3 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' | tr -s ' ')"
            printf '  %-22s %s\n' "$t" "${hz:-（3s 内无消息 / 或被 QoS 过滤）}"
        else
            printf '  %-22s %s\n' "$t" "未出现在 ros2 topic list（无发布者？）"
        fi
    done
    echo
    echo "---- 最近 5 行日志 ----"
    [ -f "$LOG_FILE" ] && tail -n 20 "$LOG_FILE" | grep -v '^[[:space:]]*$' | tail -5 | cut -c1-190 || echo "  (无日志文件)"
    echo
    echo "---- 中断状态 ----"
    local last_pause last_resume
    last_pause="$(grep -c '\[中断\]' "$LOG_FILE" 2>/dev/null || echo 0)"
    last_resume="$(grep -c '\[恢复\]' "$LOG_FILE" 2>/dev/null || echo 0)"
    echo "  历史中断次数=$last_pause  恢复次数=$last_resume"
    return 0
}

cmd_logs() {
    local n=30 follow=0
    while [ $# -gt 0 ]; do
        case "$1" in
            -n) n="$2"; shift 2 ;;
            -f|--follow) follow=1; shift ;;
            *) shift ;;
        esac
    done
    [ -f "$LOG_FILE" ] || { err "日志文件不存在: $LOG_FILE"; exit 1; }
    if [ "$follow" = "1" ]; then tail -n "$n" -f "$LOG_FILE"; else tail -n "$n" "$LOG_FILE"; fi
}

# ---------- 入口 ----------
CMD="${1:-help}"; shift || true
case "$CMD" in
    start)        cmd_start "$@" ;;
    stop)         cmd_stop ;;
    restart)      cmd_stop; sleep 1; cmd_start "$@" ;;
    status|st)    cmd_status ;;
    logs|log)     cmd_logs "$@" ;;
    check|ck)     cmd_check ;;
    -h|--help|help) usage ;;
    *) err "未知子命令: $CMD（可用: start|stop|restart|status|logs）"; usage; exit 2 ;;
esac
