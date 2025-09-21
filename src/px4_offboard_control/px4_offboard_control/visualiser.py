#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode


class MAVROSVisualizer(Node):
    def __init__(self):
        super().__init__("mavros_visualizer")

        # QoS profile
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )

        # Subscriptions
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_cb, qos_profile_sub
        )
        self.pos_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pos_cb, qos_profile_sub
        )
        self.vel_sub = self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self.vel_cb,
            qos_profile_sub,
        )

        # Publishers
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped, "/mavros_visualizer/vehicle_pose", 10
        )
        self.vehicle_path_pub = self.create_publisher(
            Path, "/mavros_visualizer/vehicle_path", 10
        )
        self.vehicle_vel_pub = self.create_publisher(
            Marker, "/mavros_visualizer/vehicle_velocity", 10
        )

        # Path messages
        self.vehicle_path_msg = Path()
        self.vehicle_path_msg.header.frame_id = "map"
        self.setpoint_path_msg = Path()

        # Vehicle state
        self.current_pose = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.setpoint_position = np.zeros(3)

        self.trail_size = 1000

        # time stamp for the last local position update received on ROS2 topic
        self.last_local_pos_update = 0.0

        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def state_cb(self, msg):
        self.current_state = msg

    def pos_cb(self, msg: PoseStamped):
        self.current_pose[0] = msg.pose.position.x
        self.current_pose[1] = msg.pose.position.y
        self.current_pose[2] = msg.pose.position.z

    def vel_cb(self, msg: TwistStamped):
        self.last_local_pos_update = self.get_clock().now().to_msg()

        self.current_vel[0] = msg.twist.linear.x
        self.current_vel[1] = msg.twist.linear.y
        self.current_vel[2] = msg.twist.linear.z

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "arrow"
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point(x=tail[0], y=tail[1], z=tail[2])
        head_point = Point(
            x=tail[0] + dt * vector[0],
            y=tail[1] + dt * vector[1],
            z=tail[2] + dt * vector[2],
        )
        msg.points = [tail_point, head_point]
        return msg

    def append_path(self, pose_msg):
        self.vehicle_path_msg.poses.append(pose_msg)
        if len(self.vehicle_path_msg.poses) > self.trail_size:
            del self.vehicle_path_msg.poses[0]

    def cmdloop_callback(self):
        # Publish current vehicle pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.current_pose[0]
        pose_msg.pose.position.y = self.current_pose[1]
        pose_msg.pose.position.z = self.current_pose[2]
        self.vehicle_pose_pub.publish(pose_msg)

        # Update and publish path
        self.append_path(pose_msg)
        self.vehicle_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

        # Publish velocity arrow
        vel_msg = self.create_arrow_marker(1, self.current_pose, self.current_vel)
        self.vehicle_vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MAVROSVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
