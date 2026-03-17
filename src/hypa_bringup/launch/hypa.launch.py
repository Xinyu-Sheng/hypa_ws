import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    LogInfo,
    Shutdown,
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.substitutions import TextSubstitution


def generate_launch_description():
    # Configure ROS nodes for launch

    pkg_project_bringup = FindPackageShare("hypa_bringup").find("hypa_bringup")
    pkg_project_gazebo = FindPackageShare("hypa_gazebo").find("hypa_gazebo")
    pkg_project_description = FindPackageShare("hypa_description").find(
        "hypa_description"
    )
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim").find("ros_gz_sim")

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [FindPackageShare("hypa_gazebo"), "worlds", "hypa.sdf"]
                ),
                # " -v 4 ",
                " -r --gui-config ",
                PathJoinSubstitution(
                    [FindPackageShare("hypa_bringup"), "config", "hypa.config"]
                ),
            ]
            # gz_args传递给gz sim的参数，就等于gz sim xxxxxx
            # "on_exit_shutdown": "true",
        }.items(),
    )

    # Visualize in RViz
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
        # on_exit=Shutdown(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "gazebo_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": True,
                # "lazy": True 有用吗？
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=[
                    FindPackageShare("hypa_description"),
                    "GZ_SIM_RESOURCE_PATH",
                    os.path.join(pkg_project_description, "models"),
                ]
            ),
            gz_sim,
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            bridge,
            rviz,
        ]
    )
