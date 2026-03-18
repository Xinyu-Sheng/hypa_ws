import os
import sys

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

from launch.actions import ExecuteProcess


def generate_launch_description():
    # Configure ROS nodes for launch

    pkg_project_bringup = FindPackageShare("hypa_bringup").find("hypa_bringup")
    pkg_project_gazebo = FindPackageShare("hypa_gazebo").find("hypa_gazebo")
    pkg_project_description = FindPackageShare("hypa_description").find(
        "hypa_description"
    )
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim").find("ros_gz_sim")

    # 不需要设置环境变量了，因为已经在hook脚本里设置了
    # # 获取安装目录中的插件库路径
    # plugin_lib_path = os.path.join(pkg_project_gazebo, "..", "..", "lib", "hypa_gazebo")

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
                " -v 2 ",
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
                "lazy": True,  # 有用吗？
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # 不需要设置环境变量了，因为已经在hook脚本里设置了
            # # 设置环境变量，使 Gazebo 能找到 GUI 插件
            # SetEnvironmentVariable("GZ_GUI_PLUGIN_PATH", plugin_lib_path),
            # # 调试：打印环境变量验证生效
            # ExecuteProcess(
            #     cmd=["bash", "-c", f"echo GZ_GUI_PLUGIN_PATH=$GZ_GUI_PLUGIN_PATH"],
            #     output="screen",
            # ),
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
            # rosout_bridge 节点 - 实时将日志写入文件
            Node(
                package="hypa_bringup",
                executable="rosout_bridge_node",
                name="rosout_bridge",
                output="screen",
                parameters=[
                    {
                        "log_file_path": os.path.expanduser("~/.ros/hypa_logs"),
                        "log_publish_frequency": 100,
                        "min_log_level": 0,  # DEBUG
                    }
                ],
            ),
            bridge,
            rviz,
        ]
    )
