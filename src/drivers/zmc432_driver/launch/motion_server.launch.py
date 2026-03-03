from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the ROS 2 node",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="hypa",
                description="Name of the robot",
            ),
            DeclareLaunchArgument(
                "controller_ip",
                default_value="192.168.0.11",
                description="ZMC432 controller IP address",
            ),
            DeclareLaunchArgument(
                "default_units",
                default_value="1.0",
                description="Default pulse units (physical units per pulse)",
            ),
            DeclareLaunchArgument(
                "default_speed",
                default_value="10.0",
                description="Default motion speed (physical units/s)",
            ),
            DeclareLaunchArgument(
                "default_accel",
                default_value="100.0",
                description="Default acceleration (physical units/s²)",
            ),
            DeclareLaunchArgument(
                "default_decel",
                default_value="100.0",
                description="Default deceleration (physical units/s²)",
            ),
            Node(
                package="zmc432_driver",
                executable="motion_node",
                name="motion_hardware_node",
                output="screen",
                parameters=[
                    {"namespace": LaunchConfiguration("namespace")},
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"robot_name": LaunchConfiguration("robot_name")},
                    {"controller_ip": LaunchConfiguration("controller_ip")},
                    {"default_units": LaunchConfiguration("default_units")},
                    {"default_speed": LaunchConfiguration("default_speed")},
                    {"default_accel": LaunchConfiguration("default_accel")},
                    {"default_decel": LaunchConfiguration("default_decel")},
                ],
            ),
        ]
    )
