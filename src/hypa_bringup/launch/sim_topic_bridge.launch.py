#!/usr/bin/env python3
"""Launch `topic_tools` relays to bridge local topics to simulation topics.

使用方法：
  ros2 launch hypa_bringup sim_topic_bridge.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation clock"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_name", default_value="", description="Robot namespace (unused)"
        )
    )

    # Include the hypa_sim launch from hypa_gazebo so simulation is started together
    hypa_gazebo_launch = os.path.join(
        get_package_share_directory("hypa_gazebo"), "launch", "hypa_sim.launch.py"
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hypa_gazebo_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )
    )

    # Each tuple: (source_topic, target_topic, relay_node_name)
    mappings = [
        (
            "/feedback/axis0/position",
            "/sim/hypa/joint/prismatic_cable_hook_bl/cmd",
            "relay_feedback_axis0",
        ),
        (
            "/feedback/axis1/position",
            "/sim/hypa/joint/prismatic_cable_hook_br/cmd",
            "relay_feedback_axis1",
        ),
        (
            "/feedback/axis2/position",
            "/sim/hypa/joint/prismatic_cable_hook_fl/cmd",
            "relay_feedback_axis2",
        ),
        (
            "/feedback/axis3/position",
            "/sim/hypa/joint/prismatic_cable_hook_fr/cmd",
            "relay_feedback_axis3",
        ),
        (
            "/feedback/axis4/position",
            "/sim/hypa/joint/z_joint/cmd",
            "relay_feedback_axis4",
        ),
        (
            "/cmd/mimic_group1",
            "/sim/hypa/joint/secondary_link_joint/cmd",
            "relay_cmd_mimic_group1",
        ),
        (
            "/cmd/mimic_group2",
            "/sim/hypa/joint/slide_link_joint/cmd",
            "relay_cmd_mimic_group2",
        ),
    ]

    for src, dst, name in mappings:
        node = Node(
            package="topic_tools",
            executable="relay",
            name=name,
            output="screen",
            arguments=[src, dst],
            parameters=[{"use_sim_time": use_sim_time}],
        )
        ld.add_action(node)

    return ld
