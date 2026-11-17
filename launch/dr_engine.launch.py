#!/usr/bin/env python3
"""DrEngine 启动：节点 + (可选) rviz2。
用法:
  ros2 launch dr_engine dr_engine.launch.py                     # 只起节点
  ros2 launch dr_engine dr_engine.launch.py use_rviz:=true      # 同时起 rviz2
  ros2 launch dr_engine dr_engine.launch.py use_sim_time:=false # 实时上车
"""
import os

from ament_index_python.packages import get_package_share_directory  # noqa: F401  (colcon 下可用)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(PKG, "config", "dr_params.yaml")),
        Node(
            package="dr_engine",
            executable="dr_engine_node",
            name="dr_engine_node",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(PKG, "rviz", "dr_engine.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
