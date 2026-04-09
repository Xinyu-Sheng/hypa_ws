import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    UnsetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成 Gazebo 仿真启动描述。"""

    pkg_project_gazebo = FindPackageShare("hypa_gazebo").find("hypa_gazebo")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim").find("ros_gz_sim")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                os.path.join(pkg_project_gazebo, "worlds", "hypa.sdf"),
                " -v 4 ",
                " -r --gui-config ",
                os.path.join(pkg_project_gazebo, "config", "hypa.config"),
            ]
        }.items(),
    )

    wmctrl_fullscreen = RegisterEventHandler(
        OnProcessStart(
            target_action=lambda action: (
                isinstance(action, ExecuteProcess)
                and any(x in " ".join(action.cmd) for x in ["gz sim", "gz-sim"])
            ),
            on_start=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-c",
                        "for i in {1..50}; do "
                        'id=$(wmctrl -l | grep -m1 "Gazebo" | cut -d " " -f1); '
                        '[ -n "$id" ] && wmctrl -ir "$id" -b add,fullscreen && exit 0; '
                        "sleep 0.1; done",
                    ],
                    output="screen",
                ),
            ],
        )
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_gazebo, "config", "gazebo_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": True,
                "lazy": True,
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["bash", "-c", "echo GZ_GUI_PLUGIN_PATH=$GZ_GUI_PLUGIN_PATH"],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["bash", "-c", "echo GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"],
                output="screen",
            ),
            UnsetEnvironmentVariable("GZ_CONFIG_PATH"),
            gz_sim,
            # wmctrl_fullscreen,
            bridge,
        ]
    )
