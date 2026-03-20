#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HWT9053 IMU 驱动 Launch 文件

支持两种运行模式：
1. 真机模式 (use_sim_time=false): 启动 SocketCAN 接收节点并驱动 HWT9053 解析
2. 仿真模式 (use_sim_time=true): 仅启动驱动并使用仿真时钟，等待外部数据源

参数从配置文件 hwt9053_params.yaml 读取，可在 launch 命令行覆盖。

使用示例：
  # 真机模式（使用默认配置文件）
  ros2 launch hwt9053_can_driver imu_system.launch.py

  # 真机模式（自定义机器人名称）
  ros2 launch hwt9053_can_driver imu_system.launch.py robot_name:=my_robot

  # 真机模式（使用自定义配置文件）
  ros2 launch hwt9053_can_driver imu_system.launch.py params_file:=/path/to/params.yaml

  # 仿真模式（预留）
  ros2 launch hwt9053_can_driver imu_system.launch.py use_sim_time:=true robot_name:=robot
"""

import os
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


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

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时钟（true=仿真模式，false=真机模式）",
    )

    can_interface_arg = DeclareLaunchArgument(
        "can_interface", default_value="can0", description="CAN 网络接口名"
    )

    can_bus_topic_arg = DeclareLaunchArgument(
        "can_bus_topic",
        default_value="from_can_bus",
        description="CAN 接收话题名称",
    )

    # 获取参数
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    can_interface = LaunchConfiguration("can_interface")
    can_bus_topic = LaunchConfiguration("can_bus_topic")

    # SocketCAN 接收节点（真机模式）
    socket_can_receiver_node = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver",
        namespace="",
        output="screen",
        parameters=[
            {
                "interface": can_interface,
                "enable_can_fd": False,
                "interval_sec": 0.01,
                "filters": "0:0",
                "use_bus_time": False,
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("from_can_bus", can_bus_topic)],
        condition=UnlessCondition(use_sim_time),
    )

    # HWT9053 CAN 驱动节点（LifecycleNode）
    hwt9053_driver_node = LifecycleNode(
        package="hwt9053_can_driver",
        executable="hwt9053_can_driver_node",
        name="hwt9053_can_driver",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "robot_name": robot_name,
                "use_sim_time": use_sim_time,
                "can_interface": can_interface,
                "can_bus_topic": can_bus_topic,
            },
        ],
    )

    configure_receiver_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(socket_can_receiver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    register_receiver_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=socket_can_receiver_node,
            on_start=[configure_receiver_event],
        ),
        condition=UnlessCondition(use_sim_time),
    )

    activate_receiver_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(socket_can_receiver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    register_receiver_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=socket_can_receiver_node,
            goal_state="inactive",
            entities=[activate_receiver_event],
        ),
        condition=UnlessCondition(use_sim_time),
    )

    # 自动 configure -> activate（Humble 无 autostart）
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(hwt9053_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    register_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=hwt9053_driver_node,
            on_start=[configure_event],
        ),
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(hwt9053_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    register_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=hwt9053_driver_node,
            goal_state="inactive",
            entities=[activate_event],
        ),
    )

    # 在机器人命名空间内组织节点
    hwt9053_driver_group = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            socket_can_receiver_node,
            hwt9053_driver_node,
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            robot_name_arg,
            use_sim_time_arg,
            can_interface_arg,
            can_bus_topic_arg,
            register_receiver_configure,
            register_receiver_activate,
            register_configure,
            register_activate,
            hwt9053_driver_group,
        ]
    )
