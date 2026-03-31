import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成系统级启动描述。"""

    pkg_project_bringup = FindPackageShare("hypa_bringup").find("hypa_bringup")
    pkg_project_gazebo = FindPackageShare("hypa_gazebo").find("hypa_gazebo")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "hypa.launch.py")
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project_bringup, "config", "hypa.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            gazebo_launch,
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            Node(
                package="hypa_bringup",
                executable="rosout_bridge_node",
                name="rosout_bridge",
                output="screen",
                parameters=[
                    {
                        "log_file_path": os.path.expanduser("~/.ros/hypa_logs"),
                        "log_publish_frequency": 100,
                        "min_log_level": 0,
                    }
                ],
            ),
            rviz,
        ]
    )
