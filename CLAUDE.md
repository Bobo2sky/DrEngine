# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概览

DrEngine：J6 车辆的纯 DR（Dead Reckoning，航位推算）节点。ROS 2 humble (C++17)。
输入 `/m22/imu/data_raw`（姿态 AHRS + 积分节拍）+ `/ins/velocity`（车速），输出 `/tf`(odom→base_link)、`/dr_odom`、`/dr_path`、`/total_mileage`。

**独立 CMake 工程，不走 colcon**——用 `script/build.sh` 直编，产物 `build/dr_engine_node`。

## 常用命令

```bash
# 编译（脚本自动 source /opt/ros/humble + /workspace/J6/vehiche/install，无需手动 source）
bash script/build.sh

# 运行（后台常驻；参数全部来自 config/dr_params.yaml）
./script/run.sh start          # 也可 stop | restart | status | logs -f | check
./script/run.sh start -p publish_rate_hz:=20.0   # 临时覆盖参数

# bag 回放验证（回放话题自动取自参数文件的 imu_topic/speed_topic；
# play_bag.sh 不存在时自动回退 ros2 bag play）
BAG=/workspace/J6/j6/rosbag2_2026_11_06-20_48_30 ./script/run.sh start

# ARM(J6 车端) 交叉编译：docker 镜像 + ~/j6-dev 依赖，产出 build-arm/dr_engine_node
./script/build_arm.sh [--msgs]
```

无单元测试，验证方式是 bag 回放（见下）。日志：节点写 `log/dr.text`，stdout 在 `run/node.out`（注意 `[退出]` 统计行只在 stdout，不落盘）。

## 架构

三层，自上而下：

- **`src/dr_node.cpp`** — ROS 薄壳：参数加载、订阅/发布、定时器、超时中断逻辑。关键点：
  - **超时判定用墙钟**（`steady_clock`，`wall_s()`），不能用 ROS 时间——回放时 `use_sim_time=true` 数据断流后 /clock 停走，用 ROS 时间算年龄会永远触发不了中断。
  - IMU 与车速**同时**超时 ⇒ 停止递推、TF 位置归零（`pause_on_both_timeout`）。
  - TF/odom 的 stamp 取 IMU 报文头时间，非节点时间。
- **`src/dr_engine.cpp` + `include/dr_engine/`** — 无 ROS 依赖的 DR 核心：
  - 姿态：`src/madgwick/`（内置 Fusion/Madgwick 6 轴，无磁力计 ⇒ yaw 无绝对基准，只有相对漂移）。
  - 速度：**大小**来自车速源，方向由姿态投影到世界系 `v = speed·(cosP·cosY, cosP·sinY, sinP)`。
  - 位置：只对速度积分 `pos += v·dt`；**加速度绝不参与位置积分**（纯惯性二重积分几十秒发散，这是该 DR 长时不漂的关键）。
- **`src/log_sink.cpp`** — stdout + 文件双写。

## 车速源语义（重要）

- `/ins/velocity` 是 `m22_road_msgs/msg/InsVelocity`：INS 组合导航 ENU 速度矢量。车速取 `velocity_enu` 的**水平幅值** `hypot(x, y)`，口径与 vehiche 仓库参考实现（`imu_speed_tf/sources.py`、`speed/compare_speed.py`）一致。
- `velocity_body` 是设备系（前左上）且未做安装外参转换，**不用**。
- InsVelocity **无挡位信息** ⇒ 无倒挡取负（旧 chassis_info 时代的 `gear_reverse_fix` 已移除）。已知局限：倒车段 DR 按前进积分。
- **数据陷阱**：bag `rosbag2_2026_11_06-20_48_30` 中 `/ins/velocity` 全为 0（speed_summary/tf_summary 证实），回放它只能验证接收管道（speed=0、里程不增）；轨迹验证需用含非零 ins 数据的 bag 或实车。

## 消息包依赖（m22_road_msgs）

- 来源：`/workspace/J6/vehiche/install`（先 `source /workspace/J6/vehiche/install/setup.bash`；build.sh/run.sh 已自动做）。
- CMake 走标准 `find_package(m22_road_msgs)`。链接只挑每个消息包的 `__rosidl_typesupport_cpp` target（不用 `${<pkg>_TARGETS`，那会把 `__generator_py` 也拉进来，其 INTERFACE 写死构建机的 libpython 绝对路径，车端链接失败）。x86 vehiche install 有 aggregate `m22_road_msgs::m22_road_msgs`，J6 humble isolated 布局只有 `m22_road_msgs::m22_road_msgs__rosidl_typesupport_cpp`，故按 target 是否存在挑选。
- RPATH 机制：CMakeLists 由 `m22_road_msgs_DIR` 回推包前缀，把 `<包目录>/lib` 写进产物（DT_RPATH，`--disable-new-dtags`），使二进制**不 source 也能跑**。目标机部署用 `-DIMUDR_DEPLOY_MSGDIR=<目标机 m22_road_msgs 包目录>` 覆盖。
- **`script/build_arm.sh` 已废弃**：仍传 `-DMSG_DIR`（该机制已从 CMakeLists 移除）且假定 ARM 覆盖层是 chassis_msgs。J6 车端改为**原生编译**（见下「J6 原生编译运行」），不再用交叉编译。

## J6 原生编译运行（推荐，已验证）

J6（`ssh j6_2`，aarch64 / Debian12 / ROS humble，colcon isolated 布局）**没有自带编译器/cmake**。已在 `/data/toolchain` 部署一套 relocatable 工具链（dpkg 解包，免 apt install 绕开 overlay 里的 broken 依赖），含 g++-12/cmake/make 及 spdlog/fmt/yaml-cpp/openssl/eigen 等开发包。`/data/toolchain/env.sh` 一键配好 sysroot/LD_LIBRARY_PATH/CMAKE_PREFIX_PATH。

```bash
# 车端编译（源码在 /data/DrEngine）
source /opt/ros/humble/setup.bash
source /app/road/m22_road/setup.bash          # 提供 m22_road_msgs（InsVelocity）
source /data/toolchain/env.sh                  # 工具链 + 第三方 cmake 包
cmake -S /data/DrEngine -B /data/DrEngine/build -DCMAKE_BUILD_TYPE=Release \
      -DIMUDR_DEPLOY_MSGDIR=/app/road/m22_road/install/m22_road_msgs
cmake --build /data/DrEngine/build -j6        # 产物 build/dr_engine_node (aarch64)

# 车端运行（用 /data/DrEngine/run_j6.sh，source ROS+m22_road 并 export RMW_IMPLEMENTATION=rmw_fastrtps_cpp）
/data/DrEngine/run_j6.sh
```

注意：
- J6 用**自定义 RMW** `rmw_communication`（hobot 共享内存中间件），但 m22 驱动跑的是 `rmw_fastrtps_cpp` ⇒ 运行节点与 `ros2` CLI 都要 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，否则 `ros2 topic list` 只见 `/parameter_events`。
- 消息导出 target 的 INTERFACE 引用构建机 libpython 路径（`/opt/j6-sysroot/...`、`/sysroot/...libpython3.11.so`），已软链到车端 `libpython3.11.so.1`。
- 源码改动需重新同步到车端：`scp <file> j6_2:/data/DrEngine/...`。
- 验证口径同 x86 bag 回放：`订阅 .../ins/velocity (m22_road_msgs/InsVelocity)`、`[车速] 首帧`、`车速过期=0`、里程递增（注意实车 ins/velocity 若低速恒 0，先确认 INS 模块是否输出）。输出 `/dr_odom`50Hz `/dr_path`1Hz `/total_mileage`50Hz，`road_cleaning_recorder` 已在订阅。

## 配置与脚本约定

- `config/dr_params.yaml` 是唯一参数入口（节点名 `dr_engine_node`）；`run.sh` 也从它读 `imu_topic`/`speed_topic`（健康检查、bag 回放话题），改话题只需改这里。
- QoS 默认 best_effort（`qos_reliable: false`）；ins/velocity 约 10Hz，`speed_timeout_s: 0.3` 够用。
- `run.sh start` 起 bag 前固定 `sleep 12` 等 mcap 索引 ⇒ 节点会漏掉 bag 开头约 12s 数据（统计帧数时注意）。
- 验证参考数据：`/workspace/J6/vehiche/output/speed/speed_summary.csv` 是同 bag 各速度源（psrvel/bestgnssvel/ins/chassis）的全量积分里程/速度范围，可作交叉基准。

## bag 回放验证要点

用 `/workspace/J6/j6/rosbag2_2026_11_06-20_48_30`（351s，`/ins/velocity` 3504 帧 @10Hz 但**全为 0**；同 bag psrvel 全量积分 441.8m、chassis 435.0m 可作对照）：

- 启动日志应出现 `订阅 /ins/velocity (m22_road_msgs/InsVelocity)`，无"尚未收到"告警。
- `[车速] 首帧` 打印 speed + ins/gnss_status + alarm；该 bag 预期 speed=0.0000、里程恒 0（数据全零，非代码问题）。
- 有非零 ins 数据时：周期日志 `speed=` 随车变化、里程递增、`车速过期=0`；交叉验证 `ros2 topic echo /ins/velocity --once` 的 hypot(velocity_enu.x, y) 与节点日志 speed 一致。
- bag 播完后应触发 `[中断]`（IMU+车速同时超时、位置归零）——这是预期行为，不是 bug。
