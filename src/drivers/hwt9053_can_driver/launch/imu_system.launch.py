#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Xinyu Sheng
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
IMU 系统综合 Launch 文件

支持两种数据源：
1. 仿真模式 (use_sim=true): 从 Gazebo 获取 IMU 数据
2. 真机模式 (use_sim=false): 从 CAN 总线获取 HWT9053 IMU 数据

使用示例：
  # 仿真模式
  ros2 launch hwt9053_can_driver imu_system.launch.py use_sim:=true robot_name:=robot

  # 真机模式
  ros2 launch hwt9053_can_driver imu_system.launch.py use_sim:=false can_interface:=can0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明launch参数
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="robot", description="机器人名称，用于命名空间隔离"
    )

    can_interface_arg = DeclareLaunchArgument(
        "can_interface", default_value="can0", description="CAN 接口名称（例如 can0）"
    )

    imu_frame_id_arg = DeclareLaunchArgument(
        "imu_frame_id", default_value="imu_link", description="IMU 传感器的 TF frame id"
    )

    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="使用 Gazebo 仿真 IMU 数据源（true/false）",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时钟（需要 Gazebo 启动）",
    )

    log_debug_arg = DeclareLaunchArgument(
        "log_debug", default_value="false", description="启用调试日志输出"
    )

    # 获取参数
    robot_name = LaunchConfiguration("robot_name")
    can_interface = LaunchConfiguration("can_interface")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_debug = LaunchConfiguration("log_debug")

    # 获取 hwt9053_can_driver 包的路径
    hwt9053_package_path = FindPackageShare("hwt9053_can_driver")

    # 真机模式：启动 HWT9053 CAN 驱动
    hwt9053_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [hwt9053_package_path, "launch", "hwt9053_driver.launch.py"]
            )
        ),
        launch_arguments={
            "robot_name": robot_name,
            "can_interface": can_interface,
            "imu_frame_id": imu_frame_id,
            "use_sim_time": use_sim_time,
            "log_debug": log_debug,
        }.items(),
        condition=UnlessCondition(use_sim),
    )

    return LaunchDescription(
        [
            robot_name_arg,
            can_interface_arg,
            imu_frame_id_arg,
            use_sim_arg,
            use_sim_time_arg,
            log_debug_arg,
            hwt9053_driver_launch,
        ]
    )
