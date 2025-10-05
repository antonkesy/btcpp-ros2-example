import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    trees_path = str(
        os.path.join(
            get_package_share_directory("coordinator"),
            "trees",
        ),
    )

    coordinator_node = Node(
        package="coordinator",
        executable="coordinator",
        parameters=[
            {"trees_path": trees_path},
        ],
    )

    ld.add_action(coordinator_node)
    return ld
