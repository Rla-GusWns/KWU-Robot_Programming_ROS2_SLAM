from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0.0", "0.0", "0.0", "map", "odom"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_footprint",
                    "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "-0.01",
                    "0",
                    "0.06",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_link",
                    "laser",
                ],
            ),
        ]
    )
