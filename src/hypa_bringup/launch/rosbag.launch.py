#!/usr/bin/env python3
"""独立启动 joint_states 的 rosbag 录制进程。"""

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


def _resolve_unique_output_prefix(_output_prefix: str) -> str:
    prefix_path = Path(_output_prefix).expanduser()
    prefix_path.parent.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    candidate = prefix_path.parent / f"{prefix_path.name}_{timestamp}"

    unique_candidate = candidate
    suffix = 1
    while unique_candidate.exists():
        unique_candidate = (
            prefix_path.parent / f"{prefix_path.name}_{timestamp}_{suffix}"
        )
        suffix += 1

    return str(unique_candidate)


def _launch_rosbag(context, *_args, **_kwargs):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    topic_name = LaunchConfiguration("topic_name").perform(context).strip("/")
    output_prefix = LaunchConfiguration("output_prefix").perform(context)
    resolved_output_prefix = _resolve_unique_output_prefix(output_prefix)

    if namespace:
        topic = f"/{namespace}/{topic_name}"
    else:
        topic = f"/{topic_name}"

    return [
        LogInfo(msg=f"启动 rosbag 录制: {resolved_output_prefix} -> {topic}"),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-o",
                resolved_output_prefix,
                "--topics",
                topic,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="hypa",
                description="机器人命名空间。",
            ),
            DeclareLaunchArgument(
                "topic_name",
                default_value="joint_states",
                description="要录制的话题名。",
            ),
            DeclareLaunchArgument(
                "output_prefix",
                default_value="/tmp/zmotion_joint_states_bag",
                description="rosbag 输出前缀。",
            ),
            OpaqueFunction(function=_launch_rosbag),
        ]
    )
