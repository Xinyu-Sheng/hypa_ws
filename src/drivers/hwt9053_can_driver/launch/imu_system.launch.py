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

使用示例：
  # 真机模式
  ros2 launch hwt9053_can_driver imu_system.launch.py robot_name:=robot can_interface:=can0

  # 仿真模式（预留）
  ros2 launch hwt9053_can_driver imu_system.launch.py use_sim:=true robot_name:=robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # 声明 launch 参数
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
        description="使用仿真 IMU 数据源（true/false，仿真模式预留）",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时钟（需要 Gazebo 启动）",
    )

    log_debug_arg = DeclareLaunchArgument(
        "log_debug", default_value="false", description="启用调试日志输出"
    )

    accel_covariance_arg = DeclareLaunchArgument(
        "accel_covariance",
        default_value="3.4e-5",
        description="线性加速度协方差 (m/s^2)^2",
    )

    gyro_covariance_arg = DeclareLaunchArgument(
        "gyro_covariance",
        default_value="5.8e-8",
        description="角速度协方差 (rad/s)^2",
    )

    # 获取参数
    robot_name = LaunchConfiguration("robot_name")
    can_interface = LaunchConfiguration("can_interface")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_debug = LaunchConfiguration("log_debug")
    accel_covariance = LaunchConfiguration("accel_covariance")
    gyro_covariance = LaunchConfiguration("gyro_covariance")

    # HWT9053 CAN 驱动节点
    hwt9053_driver_node = Node(
        package="hwt9053_can_driver",
        executable="hwt9053_can_driver_node",
        output="screen",
        parameters=[
            {
                "robot_name": robot_name,
                "can_interface": can_interface,
                "imu_frame_id": imu_frame_id,
                "use_sim_time": use_sim_time,
                "log_debug": log_debug,
                "accel_covariance": accel_covariance,
                "gyro_covariance": gyro_covariance,
                "imu_topic_name": "imu/data",
                "can_bus_topic": "from_can_bus",
            }
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
            robot_name_arg,
            can_interface_arg,
            imu_frame_id_arg,
            use_sim_arg,
            use_sim_time_arg,
            log_debug_arg,
            accel_covariance_arg,
            gyro_covariance_arg,
            hwt9053_driver_group,
        ]
    )
