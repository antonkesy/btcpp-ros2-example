from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Actions launches first
    actions_cpp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("actions_cpp"),
                    "launch",
                    "action_server.launch.py",
                ],
            ),
        ),
    )

    actions_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("actions_py"),
                    "launch",
                    "action_server.launch.py",
                ],
            ),
        ),
    )

    # Coordinator and supervisor launches after a small delay to ensure actions are up
    coordinator_launch = TimerAction(
        period=1.0,
        actions=[
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
        ],
    )

    robot_supervisor_launch = TimerAction(
        period=1.0,
        actions=[
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

    return LaunchDescription(
        [
            actions_cpp_launch,
            actions_py_launch,
            coordinator_launch,
            robot_supervisor_launch,
        ],
    )
