cmake_minimum_required(VERSION 3.16)
project(dr_engine CXX)

# ============================================================================
#  DrEngine —— 纯 DR 工程（x86 直编 / aarch64(J6) 交叉编译通用）
#  输入: /imu/data + /chassis_info
#  输出: /tf(odom->base_link) + /dr_odom + /dr_path + /total_mileage
#
#  参考 src/localization/hg-slam/CMakeLists.txt 的做法：
#    · 先按 CMAKE_SYSTEM_PROCESSOR 判 ARM/x86；
#    · 消息包（chassis_msgs）用 MSG_DIR 覆盖层定位 —— 不依赖 source setup.bash，
#      交叉编译时用 -DMSG_DIR=<aarch64 覆盖层> 指到 ARM 版本；
#    · 显式链接 typesupport 动态库并写入 RPATH ⇒ 产物在本机/目标机都能直接运行。
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

# ---- 消息包覆盖层（chassis_msgs）----
#   默认值按架构区分（均可用 -DMSG_DIR=... 覆盖）：
#     · ARM (车端 J6 原生编译)  : /data/runtime/ros_msg/install
#     · x86 (开发机)            : 本仓库 app/common/install
if(ARM)
    set(MSG_DIR_DEFAULT "/data/runtime/ros_msg/install")
else()
    set(MSG_DIR_DEFAULT "${PROJECT_SOURCE_DIR}/../../../app/common/install")
endif()
set(MSG_DIR "${MSG_DIR_DEFAULT}" CACHE PATH "消息包覆盖层根目录（含 chassis_msgs/<include,lib>）")
get_filename_component(MSG_DIR "${MSG_DIR}" ABSOLUTE)
message(STATUS "DrEngine: MSG_DIR = ${MSG_DIR}  (ARM=${ARM}, 默认值 ${MSG_DIR_DEFAULT})")

if(NOT EXISTS "${MSG_DIR}/chassis_msgs")
    if(ARM)
        set(_alt "${PROJECT_SOURCE_DIR}/../../../app/common/install")
    else()
        set(_alt "/data/runtime/ros_msg/install")
    endif()
    message(FATAL_ERROR "在 MSG_DIR 下找不到 chassis_msgs：${MSG_DIR}/chassis_msgs\n"
                        "  当前架构默认值 : ${MSG_DIR_DEFAULT}\n"
                        "  另一套可能路径 : ${_alt}\n"
                        "  提示: 用 -DMSG_DIR=<路径> 指定，例如车端原生编译 -DMSG_DIR=/data/runtime/ros_msg/install")
endif()

# ---- ROS 2 依赖 ----
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- 消息包链接：直接走 MSG_DIR 覆盖层（与 hg-slam 同款，不依赖 source）----
include_directories(${MSG_DIR}/chassis_msgs/include/chassis_msgs)
link_directories(${MSG_DIR}/chassis_msgs/lib)

set(CHASSIS_LIBS "")
foreach(part generator_c typesupport_c typesupport_cpp typesupport_fastrtps_cpp typesupport_introspection_cpp)
    set(_so "${MSG_DIR}/chassis_msgs/lib/libchassis_msgs__rosidl_${part}.so")
    if(EXISTS "${_so}")
        list(APPEND CHASSIS_LIBS "${_so}")
    elseif(part STREQUAL "typesupport_cpp")
        message(FATAL_ERROR "缺少 ${_so}\n"
                            "  提示: MSG_DIR 指向的覆盖层架构不对（x86 顶层 / aarch64）时请换 -DMSG_DIR=...")
    endif()
endforeach()
message(STATUS "DrEngine: chassis_msgs 库 = ${CHASSIS_LIBS}")

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

target_link_libraries(dr_engine_node
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
    ${CHASSIS_LIBS}
    ${rcutils_TARGETS}
    Eigen3::Eigen
)

# ---- RPATH：把消息覆盖层 / ROS 库目录写进产物 ----
#   这样 `./build/dr_engine_node` 即使不 source 覆盖层，也不会再报
#   "libchassis_msgs__rosidl_generator_c.so: cannot open shared object file"。
# BUILD_RPATH 用编译时的 MSG_DIR；部署到目标机（如 J6: /opt/work/app/common/install）时
# 用 -DIMUDR_DEPLOY_MSGDIR=... 让产物里的 RPATH 指向目标机真实路径。
if(ARM)
    set(_deploy_default "/data/runtime/ros_msg/install")
else()
    set(_deploy_default "${MSG_DIR}")
endif()
set(IMUDR_DEPLOY_MSGDIR "${_deploy_default}"
    CACHE PATH "目标机上的消息覆盖层根目录（写入产物 RPATH）")
set(IMUDR_RPATH "${IMUDR_DEPLOY_MSGDIR}/chassis_msgs/lib")
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
