cmake_minimum_required(VERSION 3.16)
project(dr_engine CXX)

# ============================================================================
#  DrEngine —— 纯 DR 工程（x86 直编 / aarch64(J6) 交叉编译通用）
#  输入: /m22/imu/data_raw + /ins/velocity (m22_road_msgs/InsVelocity)
#  输出: /tf(odom->base_link) + /dr_odom + /dr_path + /total_mileage
#
#  消息包（m22_road_msgs）走标准 find_package：
#    · 编译前 source 消息层 install（开发机: /workspace/J6/vehiche/install/setup.bash，
#      build.sh 已自动加该候选；车端可由 CMAKE_PREFIX_PATH / AMENT_PREFIX_PATH 指到对应 install）；
#    · 仍显式把 m22_road_msgs 的 lib 目录写进产物 RPATH ⇒ 不 source 也能直接运行。
# ============================================================================

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ---- 架构判定（与 hg-slam 一致）----
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm|ARM")
    set(ARM ON)
    message(STATUS "DrEngine: Detected ARM architecture (${CMAKE_SYSTEM_PROCESSOR})")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|i386|AMD64|amd64")
    set(ARM OFF)
    message(STATUS "DrEngine: Detected x86 architecture (${CMAKE_SYSTEM_PROCESSOR})")
else()
    message(WARNING "DrEngine: Unknown architecture ${CMAKE_SYSTEM_PROCESSOR}, defaulting to x86")
    set(ARM OFF)
endif()

# ---- 消息包（m22_road_msgs）----
#   前提：cmake 前 source 过消息层 install（含 m22_road_msgs），
#   开发机即: source /workspace/J6/vehiche/install/setup.bash（build.sh 自动做）
find_package(m22_road_msgs REQUIRED)

# ---- ROS 2 依赖 ----
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(dr_engine_node
    src/dr_node.cpp
    src/dr_engine.cpp
    src/log_sink.cpp
    src/madgwick/FusionAhrs.cpp
    src/madgwick/FusionCompass.cpp
    src/madgwick/FusionOffset.cpp
)

target_include_directories(dr_engine_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dr_engine
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dr_engine/madgwick   # 让 FusionXxx.cpp 里的 "FusionXxx.h" 直接可解析
)

# 消息包只链 C++ typesupport target。直接用 ${<pkg>_TARGETS} 会把 generator_py
# 也拉进来（其 INTERFACE 写死了构建机的 libpython 绝对路径，车端无此文件 ⇒ 链接失败）。
# 因此逐个挑选 <pkg>::<pkg>__rosidl_typesupport_cpp。
set(_msg_cpp_targets)
foreach(_pkg sensor_msgs nav_msgs geometry_msgs tf2_msgs m22_road_msgs)
    if(TARGET ${_pkg}::${_pkg}__rosidl_typesupport_cpp)
        list(APPEND _msg_cpp_targets ${_pkg}::${_pkg}__rosidl_typesupport_cpp)
    else()
        message(FATAL_ERROR "${_pkg} 缺少 __rosidl_typesupport_cpp target")
    endif()
endforeach()

target_link_libraries(dr_engine_node
    rclcpp::rclcpp
    ${_msg_cpp_targets}
    ${rcutils_TARGETS}
    Eigen3::Eigen
)

# ---- RPATH：把消息层(m22_road_msgs) / ROS 库目录写进产物 ----
#   这样 `./build/dr_engine_node` 即使不 source 消息层 install，也不会再报
#   "libm22_road_msgs__rosidl_typesupport_cpp.so: cannot open shared object file"。
#   m22_road_msgs_DIR = <包前缀>/share/m22_road_msgs/cmake（isolated/merged 布局均适用，
#   回推三级即包前缀，其下 lib/ 就是消息库）。部署到目标机（如 J6）时用
#   -DIMUDR_DEPLOY_MSGDIR=<目标机 m22_road_msgs 包目录> 让 RPATH 指向真实路径。
get_filename_component(_m22_pkg_dir  "${m22_road_msgs_DIR}" DIRECTORY)   # <prefix>/share/m22_road_msgs
get_filename_component(_m22_share    "${_m22_pkg_dir}" DIRECTORY)        # <prefix>/share
get_filename_component(_m22_prefix   "${_m22_share}" DIRECTORY)          # <prefix>（其下 lib/ 为消息库）
set(IMUDR_DEPLOY_MSGDIR "${_m22_prefix}"
    CACHE PATH "目标机上的 m22_road_msgs 包目录（其下含 lib/，写入产物 RPATH）")
set(IMUDR_RPATH "${IMUDR_DEPLOY_MSGDIR}/lib")
foreach(_rl "/opt/ros/$ENV{ROS_DISTRO}/lib" "/opt/ros/humble/lib" "/opt/ros/jazzy/lib")
    if(EXISTS "${_rl}")
        list(APPEND IMUDR_RPATH "${_rl}")
    endif()
endforeach()
list(REMOVE_DUPLICATES IMUDR_RPATH)
# --disable-new-dtags ⇒ 生成 DT_RPATH（而非 DT_RUNPATH），使 RPATH 对【间接依赖】也生效，
# 这样即使完全不 source 任何 setup.bash，`./build/dr_engine_node` 也能跑起来。
target_link_options(dr_engine_node PRIVATE -Wl,--disable-new-dtags)

set_target_properties(dr_engine_node PROPERTIES
    BUILD_RPATH "${IMUDR_RPATH}"
    INSTALL_RPATH "${IMUDR_RPATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
)
message(STATUS "DrEngine: RPATH = ${IMUDR_RPATH}")

install(TARGETS dr_engine_node DESTINATION lib/${PROJECT_NAME})
