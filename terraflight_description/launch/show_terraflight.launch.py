from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, Shutdown
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    return LaunchDescription(
        [
            SetLaunchConfiguration(
                name="rviz_terraflight_file",
                value=[
                    FindPackageShare("terraflight_description"),
                    TextSubstitution(text="/config/view_terraflight.rviz"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=['0','0','0','0','0','0', 'odom','base_footprint'], # x was at 0.254
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
            ),
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
                                ),
                            ]
                        )
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",

                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_terraflight_file"),
                    "-f",
                    "map"
                ],
                on_exit=Shutdown(),
            ),
        ]
    )
