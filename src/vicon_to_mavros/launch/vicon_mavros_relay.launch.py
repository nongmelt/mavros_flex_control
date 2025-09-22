#!/usr/bin/env python3

"""
Launch file for Vicon to MAVROS relay node

This launch file starts the relay node with configurable parameters
and optionally starts the Vicon bridge and MAVROS nodes.

Usage:
    ros2 launch vicon_mavros_relay vicon_mavros_relay.launch.py
    
    # With custom parameters:
    ros2 launch vicon_mavros_relay vicon_mavros_relay.launch.py \
        vicon_subject_name:=quadcopter \
        vicon_segment_name:=quadcopter \
        publish_rate_hz:=50.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the Vicon-MAVROS relay system."""

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "vicon_subject_name",
                default_value="whiskerdrone",
                description="Name of the Vicon subject to track",
            ),
            DeclareLaunchArgument(
                "vicon_segment_name",
                default_value="whiskerdrone",
                description="Name of the Vicon segment to track",
            ),
            DeclareLaunchArgument(
                "vicon_tf_namespace",
                default_value="vicon",
                description="TF namespace used by the Vicon bridge",
            ),
            DeclareLaunchArgument(
                "world_frame_id",
                default_value="map",
                description="World frame ID for pose messages",
            ),
            DeclareLaunchArgument(
                "base_link_frame_id",
                default_value="base_link",
                description="Base link frame ID for the vehicle",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Whether to publish TF transforms for debugging",
            ),
            DeclareLaunchArgument(
                "mavros_vision_pose_topic",
                default_value="/mavros/vision_pose/pose",
                description="MAVROS vision pose topic name",
            ),
            DeclareLaunchArgument(
                "position_offset_x",
                default_value="0.0",
                description="X position offset in meters",
            ),
            DeclareLaunchArgument(
                "position_offset_y",
                default_value="0.0",
                description="Y position offset in meters",
            ),
            DeclareLaunchArgument(
                "position_offset_z",
                default_value="0.0",
                description="Z position offset in meters",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="100.0",
                description="Publishing rate in Hz",
            ),
            DeclareLaunchArgument(
                "vicon_host_name",
                default_value="192.168.0.50:801",
                description="Vicon DataStream server hostname:port",
            ),
            Node(
                package="vicon_bridge",
                executable="vicon_bridge",
                name="vicon_bridge",
                output="screen",
                parameters=[
                    {
                        "host_name": LaunchConfiguration("vicon_host_name"),
                        "stream_mode": "ServerPush",
                        "publish_specific_segment": True,
                        "target_subject_name": LaunchConfiguration(
                            "vicon_subject_name"
                        ),
                        "target_segment_name": LaunchConfiguration(
                            "vicon_segment_name"
                        ),
                        "world_frame_id": LaunchConfiguration("world_frame_id"),
                        "tf_namespace": LaunchConfiguration("vicon_tf_namespace"),
                    }
                ],
            ),
            # Main relay node
            Node(
                package="vicon_to_mavros",
                executable="relay_node",
                name="relay_node",
                output="screen",
                parameters=[
                    {
                        "vicon_subject_name": LaunchConfiguration("vicon_subject_name"),
                        "vicon_segment_name": LaunchConfiguration("vicon_segment_name"),
                        "vicon_tf_namespace": LaunchConfiguration("vicon_tf_namespace"),
                        "world_frame_id": LaunchConfiguration("world_frame_id"),
                        "base_link_frame_id": LaunchConfiguration("base_link_frame_id"),
                        "publish_tf": LaunchConfiguration("publish_tf"),
                        "mavros_vision_pose_topic": LaunchConfiguration(
                            "mavros_vision_pose_topic"
                        ),
                        "position_offset_x": LaunchConfiguration("position_offset_x"),
                        "position_offset_y": LaunchConfiguration("position_offset_y"),
                        "position_offset_z": LaunchConfiguration("position_offset_z"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                    }
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
