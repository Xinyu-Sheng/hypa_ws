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
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    pkg_dir = get_package_share_directory("zmc432_driver")
    default_params_file = os.path.join(pkg_dir, "config", "zmc432_params.yaml")

    # 声明 launch 参数
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file to use",
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="robot", description="机器人名称，用于命名空间隔离"
    )

    # 获取参数
    robot_name = LaunchConfiguration("robot_name")

    motion_node = LifecycleNode(
        package="zmc432_driver",
        executable="motion_node",
        name="motion_hardware_node",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
        ],
    )

    # Humble does not support LifecycleNode autostart; simulate it via events.
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(motion_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    register_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=motion_node,
            on_start=[configure_event],
        )
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(motion_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    register_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=motion_node,
            goal_state="inactive",
            entities=[activate_event],
        )
    )

    # 在机器人命名空间内组织节点
    motion_node_group = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            motion_node,
        ]
    )

    return LaunchDescription(
        [
            params_file_arg,
            robot_name_arg,
            register_configure,
            register_activate,
            motion_node_group,
        ]
    )
