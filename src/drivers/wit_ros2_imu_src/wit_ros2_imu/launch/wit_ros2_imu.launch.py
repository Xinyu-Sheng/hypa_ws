from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成 WIT IMU 驱动启动描述。"""

    default_params = PathJoinSubstitution(
        [FindPackageShare("wit_ros2_imu"), "config", "wit_ros2_imu.yaml"]
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="hypa",
        description="WIT IMU 节点命名空间",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="WIT IMU 参数文件路径",
    )

    imu_node = Node(
        package="wit_ros2_imu",
        executable="wit_ros2_imu",
        name="imuDriverNode",
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
            params_file_arg,
            imu_node,
        ]
    )
