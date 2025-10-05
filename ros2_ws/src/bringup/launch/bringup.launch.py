from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("actions_cpp"),
                            "launch",
                            "action_server.launch.py",
                        ],
                    ),
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("actions_py"),
                            "launch",
                            "action_server.launch.py",
                        ],
                    ),
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("coordinator"),
                            "launch",
                            "coordinator.launch.py",
                        ],
                    ),
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("robot_supervisor"),
                            "launch",
                            "robot_supervisor.launch.py",
                        ],
                    ),
                ),
            ),
        ],
    )
