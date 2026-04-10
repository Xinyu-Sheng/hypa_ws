#!/usr/bin/env python3
"""Launch `topic_tools` relays to bridge local topics to simulation topics.

使用方法：
  ros2 launch hypa_bringup sim_topic_bridge.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Each tuple: (source_topic, target_topic, relay_node_name)
    mappings = [
        (
            "feedback/axis0/position",
            "/sim/hypa/joint/prismatic_cable_hook_bl/cmd",
            "relay_feedback_axis0",
        ),
        (
            "feedback/axis1/position",
            "/sim/hypa/joint/prismatic_cable_hook_br/cmd",
            "relay_feedback_axis1",
        ),
        (
            "feedback/axis2/position",
            "/sim/hypa/joint/prismatic_cable_hook_fl/cmd",
            "relay_feedback_axis2",
        ),
        (
            "feedback/axis3/position",
            "/sim/hypa/joint/prismatic_cable_hook_fr/cmd",
            "relay_feedback_axis3",
        ),
        (
            "feedback/axis4/position",
            "/sim/hypa/joint/z_joint/cmd",
            "relay_feedback_axis4",
        ),
        (
            "feedback/mimic_group1/position",
            "/sim/hypa/joint/secondary_link_joint/cmd",
            "relay_feedback_mimic_group1",
        ),
        (
            "feedback/mimic_group2/position",
            "/sim/hypa/joint/slide_link_joint/cmd",
            "relay_feedback_mimic_group2",
        ),
    ]

    for src, dst, name in mappings:
        node = Node(
            package="topic_tools",
            executable="relay",
            name=name,
            namespace="hypa",
            output="screen",
            arguments=[src, dst],
        )
        ld.add_action(node)

    return ld
