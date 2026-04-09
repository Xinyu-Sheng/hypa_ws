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

    mag_topic_arg = DeclareLaunchArgument(
        "mag_topic",
        default_value="imu/mag",
        description="磁场话题名（默认 imu/mag）",
    )

    publish_mag_arg = DeclareLaunchArgument(
        "publish_mag",
        default_value="True",
        description="是否发布磁场消息（True/False）",
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
            {"mag_topic": LaunchConfiguration("mag_topic")},
            {"publish_mag": LaunchConfiguration("publish_mag")},
        ],
    )

    return LaunchDescription(
        [
            namespace_arg,
            params_file_arg,
            mag_topic_arg,
            publish_mag_arg,
            imu_node,
        ]
    )
