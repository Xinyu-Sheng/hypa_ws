from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


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
            DeclareLaunchArgument(
                "motion_command_topic",
                default_value="motion_command",
                description="Topic name for motion commands",
            ),
            DeclareLaunchArgument(
                "motion_status_topic",
                default_value="motion_status",
                description="Topic name for motion status feedback",
            ),
            DeclareLaunchArgument(
                "axis_count",
                default_value="1",
                description="Number of motion axes to configure",
            ),
            DeclareLaunchArgument(
                "perform_ecat_init",
                default_value="false",
                description="Whether to run EtherCAT bus init on node startup",
            ),
            DeclareLaunchArgument(
                "ecat_slot_id",
                default_value="0",
                description="EtherCAT slot ID to use for bus init",
            ),
            DeclareLaunchArgument(
                "ecat_timeout_ms",
                default_value="5000",
                description="Timeout (ms) for EtherCAT init operations",
            ),
            LifecycleNode(
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
                    {
                        "motion_command_topic": LaunchConfiguration(
                            "motion_command_topic"
                        )
                    },
                    {"motion_status_topic": LaunchConfiguration("motion_status_topic")},
                    {"axis_count": LaunchConfiguration("axis_count")},
                    {"perform_ecat_init": LaunchConfiguration("perform_ecat_init")},
                    {"ecat_slot_id": LaunchConfiguration("ecat_slot_id")},
                    {"ecat_timeout_ms": LaunchConfiguration("ecat_timeout_ms")},
                ],
                autostart=True,
            ),
        ]
    )
