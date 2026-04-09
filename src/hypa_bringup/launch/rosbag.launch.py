#!/usr/bin/env python3
"""独立启动固定话题的 rosbag 录制进程。"""

from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
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


def _load_topics_from_yaml() -> list[str]:
    topics_path = (
        Path(get_package_share_directory("hypa_bringup"))
        / "config"
        / "rosbag_topics.yaml"
    )
    if not topics_path.is_file():
        raise FileNotFoundError(f"rosbag topic 配置文件不存在: {topics_path}")

    topics = yaml.safe_load(topics_path.read_text(encoding="utf-8")) or []
    if not isinstance(topics, list):
        raise ValueError(f"rosbag topic 配置文件必须是话题列表: {topics_path}")

    return topics


def _launch_rosbag(context, *_args, **_kwargs):
    output_prefix = LaunchConfiguration("output_prefix").perform(context)
    resolved_output_prefix = _resolve_unique_output_prefix(output_prefix)
    topics = _load_topics_from_yaml()

    return [
        LogInfo(msg=f"启动 rosbag 录制: {resolved_output_prefix} -> {topics}"),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-o",
                resolved_output_prefix,
                "--topics",
                *topics,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_prefix",
                default_value="/tmp/zmotion_bag",
                description="rosbag 输出前缀。",
            ),
            OpaqueFunction(function=_launch_rosbag),
        ]
    )
