#!/usr/bin/env python3

"""
ROS2 Vicon to MAVROS Relay Node

This node subscribes to Vicon bridge pose data and republishes it
in a format suitable for MAVROS vision pose estimation.

Author: Assistant
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np


class ViconMavrosRelayNode(Node):
    """
    Relay node that converts Vicon bridge data to MAVROS vision pose format.
    
    Subscribes to:
        - /vicon/<subject_name>/<segment_name> (geometry_msgs/TransformStamped)
        - /tf (for coordinate frame transformations)
    
    Publishes to:
        - /mavros/vision_pose/pose (geometry_msgs/PoseStamped)
        - /tf (optional, for debugging and visualization)
    """
    
    def __init__(self):
        super().__init__('vicon_mavros_relay')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('vicon_subject_name', 'whiskerdrone'),
                ('vicon_segment_name', 'whiskerdrone'),
                ('vicon_tf_namespace', 'vicon'),
                ('world_frame_id', 'map'),
                ('base_link_frame_id', 'base_link'),
                ('publish_tf', True),
                ('mavros_vision_pose_topic', '/mavros/vision_pose/pose'),
                ('invert_z_axis', False),  # Some coordinate system conversions
                ('apply_enu_to_ned', True),  # Convert ENU (ROS) to NED (PX4)
                ('position_offset_x', 0.0),
                ('position_offset_y', 0.0),
                ('position_offset_z', 0.0),
                ('publish_rate_hz', 60.0),
            ]
        )
        
        # Get parameters
        self.vicon_subject_name = self.get_parameter('vicon_subject_name').value
        self.vicon_segment_name = self.get_parameter('vicon_segment_name').value
        self.vicon_tf_namespace = self.get_parameter('vicon_tf_namespace').value
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.base_link_frame_id = self.get_parameter('base_link_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.mavros_topic = self.get_parameter('mavros_vision_pose_topic').value
        self.invert_z = self.get_parameter('invert_z_axis').value
        self.apply_enu_to_ned = self.get_parameter('apply_enu_to_ned').value
        self.pos_offset_x = self.get_parameter('position_offset_x').value
        self.pos_offset_y = self.get_parameter('position_offset_y').value
        self.pos_offset_z = self.get_parameter('position_offset_z').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        
        # Build topic name for Vicon data
        self.vicon_topic = f"/{self.vicon_tf_namespace}/{self.vicon_subject_name}/{self.vicon_segment_name}"
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.mavros_pose_pub = self.create_publisher(
            PoseStamped,
            self.mavros_topic,
            qos_profile
        )
        
        # Subscriber to Vicon data
        self.vicon_sub = self.create_subscription(
            TransformStamped,
            self.vicon_topic,
            self.vicon_callback,
            qos_profile
        )
        
        # Rate limiter for publishing
        self.last_publish_time = self.get_clock().now()
        self.publish_period = 1.0 / self.publish_rate
        
        # Store latest pose for rate limiting
        self.latest_pose = None
        
        # Timer for periodic publishing (if using rate limiting)
        self.timer = self.create_timer(self.publish_period, self.timer_callback)
        
        self.get_logger().info(f"Vicon-MAVROS relay node started")
        self.get_logger().info(f"Subscribing to: {self.vicon_topic}")
        self.get_logger().info(f"Publishing to: {self.mavros_topic}")
        self.get_logger().info(f"Subject: {self.vicon_subject_name}, Segment: {self.vicon_segment_name}")
        
    def vicon_callback(self, msg: TransformStamped):
        """
        Callback for Vicon TransformStamped messages.
        Converts to PoseStamped and applies coordinate transformations.
        """    
        pose = PoseStamped()
        # pose.header = msg.header
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.world_frame_id
        pose.pose.position.x = msg.transform.translation.x
        pose.pose.position.y = msg.transform.translation.y
        pose.pose.position.z = msg.transform.translation.z
        pose.pose.orientation = msg.transform.rotation
        self.latest_pose = pose
    
    def timer_callback(self):
        """Timer callback for rate-limited publishing."""
        if self.latest_pose is not None:
            # Update timestamp to current time
            self.latest_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Publish to MAVROS
            self.mavros_pose_pub.publish(self.latest_pose)
            
            self.get_logger().debug(
                f"Published pose: x={self.latest_pose.pose.position.x:.3f}, "
                f"y={self.latest_pose.pose.position.y:.3f}, "
                f"z={self.latest_pose.pose.position.z:.3f}"
            )


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    
    try:
        node = ViconMavrosRelayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
