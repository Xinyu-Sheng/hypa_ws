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
HWT9053 IMU 驱动 Launch 文件

支持两种运行模式：
1. 真机模式 (use_sim=false): 从 CAN 总线获取 HWT9053 IMU 数据
2. 仿真模式 (use_sim=true): 从 Gazebo 获取 IMU 数据（后续扩展）

参数从配置文件 hwt9053_params.yaml 读取，可在 launch 命令行覆盖。

使用示例：
  # 真机模式（使用默认配置文件）
  ros2 launch hwt9053_can_driver imu_system.launch.py

  # 真机模式（自定义机器人名称）
  ros2 launch hwt9053_can_driver imu_system.launch.py robot_name:=my_robot

  # 真机模式（使用自定义配置文件）
  ros2 launch hwt9053_can_driver imu_system.launch.py params_file:=/path/to/params.yaml

  # 仿真模式（预留）
  ros2 launch hwt9053_can_driver imu_system.launch.py use_sim:=true robot_name:=robot
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_dir = get_package_share_directory("hwt9053_can_driver")
    default_params_file = os.path.join(pkg_dir, "config", "hwt9053_params.yaml")

    # 声明 launch 参数
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file to use",
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="robot", description="机器人名称，用于命名空间隔离"
    )

    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="使用仿真 IMU 数据源（true/false，仿真模式预留）",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时钟（需要 Gazebo 启动）",
    )

    # 获取参数
    robot_name = LaunchConfiguration("robot_name")
    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # HWT9053 CAN 驱动节点
    hwt9053_driver_node = Node(
        package="hwt9053_can_driver",
        executable="hwt9053_can_driver_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "robot_name": robot_name,
                "use_sim_time": use_sim_time,
            },
        ],
        condition=UnlessCondition(use_sim),
    )

    # 在机器人命名空间内组织节点
    hwt9053_driver_group = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            hwt9053_driver_node,
        ],
        condition=UnlessCondition(use_sim),
    )

    return LaunchDescription(
        [
            params_file_arg,
            robot_name_arg,
            use_sim_arg,
            use_sim_time_arg,
            hwt9053_driver_group,
        ]
    )
