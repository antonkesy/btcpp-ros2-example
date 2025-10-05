import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    WEBOTS_GATEWAY = os.environ.get("WEBOTS_GATEWAY", "localhost") or "localhost"
    return LaunchDescription(
        [
            Node(
                package="robot_supervisor",
                executable="robot_supervisor",
                output="screen",
                namespace="example",
                name="robot_supervisor",
                respawn=True,
                prefix=[
                    os.environ.get("WEBOTS_HOME", "")
                    + "/webots-controller"
                    + " --protocol=tcp"
                    + f" --ip-address={WEBOTS_GATEWAY} --port=1234",
                ],
            ),
        ],
    )
