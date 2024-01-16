from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": Command(
                        [
                            ExecutableInPackage("xacro", "xacro"),
                            " ",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("terraflight_description"),
                                    "terraflight.urdf.xacro",
                                ]
                            )
                        ]
                    )
                }
            ]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        )
    ])
