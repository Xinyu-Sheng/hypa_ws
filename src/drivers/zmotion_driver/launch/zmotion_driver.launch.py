from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_params = (
        get_package_share_directory("zmotion_driver") + "/config/zmotion_driver.yaml"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace for zmotion_driver",
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to zmotion_driver parameter file",
    )

    driver_node = LifecycleNode(
        package="zmotion_driver",
        executable="zmotion_driver_node",
        name="zmotion_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"namespace": LaunchConfiguration("namespace")},
        ],
    )

    return LaunchDescription(
        [
            namespace_arg,
            params_arg,
            driver_node,
        ]
    )
