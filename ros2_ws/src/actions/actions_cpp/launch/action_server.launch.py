from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="actions_cpp",
                executable="move_server",
            ),
            Node(
                package="actions_cpp",
                executable="get_runtime_server",
            ),
            Node(
                package="actions_cpp",
                executable="is_front_clear_server",
            ),
        ]
    )
