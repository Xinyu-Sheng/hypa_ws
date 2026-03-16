from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="servo_demo_bridge",
                arguments=[
                    "/servo_joint@std_msgs/msg/Float64]gz.msgs.Double",
                    # "/model/servo_demo/joint/servo_joint/state@std_msgs/msg/Float64@gz.msgs.Double",
                ],
                output="screen",
            )
        ]
    )
