#!/usr/bin/env python3
# 该文件启动 rosout_bridge 节点，将 ROS 2 日志持久化到文件

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    生成 launch 描述

    参数：
        - log_file_path: 日志文件存储路径（默认 ~/.ros/hypa_logs）
        - log_publish_frequency: 日志发布频率（Hz，默认 100）
        - min_log_level: 最小日志等级（DEBUG=0, INFO=1, WARN=2, ERROR=4, FATAL=8，默认 0）
    """

    # 获取家目录
    home_dir = os.path.expanduser("~")
    default_log_path = os.path.join(home_dir, ".ros/hypa_logs")

    return LaunchDescription(
        [
            Node(
                package="hypa_bringup",
                executable="rosout_bridge_node",
                name="rosout_bridge",
                output="screen",
                parameters=[
                    {
                        "log_file_path": default_log_path,
                        "log_publish_frequency": 100,
                        "min_log_level": 0,  # DEBUG
                    }
                ],
            )
        ]
    )
