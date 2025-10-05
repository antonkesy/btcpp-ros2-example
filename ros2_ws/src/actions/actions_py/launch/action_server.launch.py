from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="actions_py",
                executable="print_action_server",
                output="screen",
            ),
        ],
    )
