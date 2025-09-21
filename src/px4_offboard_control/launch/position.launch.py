from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Circle trajectory node (offboard control)
            Node(
                package="px4_offboard_control",
                executable="circle",
                name="circle",
                output="screen",
                parameters=[
                    {"radius": 0.5},
                    {"omega": 0.5},
                    {"altitude": 1.0},
                ],
            ),
            # MAVROS visualizer node
            Node(
                package="px4_offboard_control",
                executable="visualiser",
                name="visualiser",
                output="screen",
            ),
        ]
    )
