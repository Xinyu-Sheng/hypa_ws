from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_params = (
        get_package_share_directory("zmotion_driver") + "/config/zmotion_driver.yaml"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="hypa",
        description="ROS namespace for zmotion_driver",
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to zmotion_driver parameter file",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )

    driver_node = LifecycleNode(
        package="zmotion_driver",
        executable="zmotion_driver_node",
        name="zmotion_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Request the lifecycle 'configure' transition for this node on startup.
    # We intentionally do NOT request 'activate' here — driver should remain
    # in 'inactive' until manually activated.
    emit_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription(
        [
            namespace_arg,
            params_arg,
            use_sim_time_arg,
            driver_node,
            emit_configure,
        ]
    )
