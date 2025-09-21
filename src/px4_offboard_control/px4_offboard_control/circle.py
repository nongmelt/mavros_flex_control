#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped


class MavrosOffboardControl(Node):

    def __init__(self):
        super().__init__("mavros_offboard_control")

        # Parameters
        self.declare_parameter("radius", 0.5)
        self.declare_parameter("omega", 0.5)
        self.declare_parameter("altitude", 1.0)

        self.radius = self.get_parameter("radius").get_parameter_value().double_value
        self.omega = self.get_parameter("omega").get_parameter_value().double_value
        self.altitude = (
            self.get_parameter("altitude").get_parameter_value().double_value
        )

        # State subscriber
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_cb, qos
        )
        self.pos_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pos_cb, qos
        )

        self.current_state = State()

        # Publisher for local position setpoints
        self.local_pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Timer for publishing setpoints
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        # Internal vars
        self.current_pose = np.zeros(3)
        self.theta = 0.0
        self.sent_setpoints = 0
        self.start_time = None
        self.landing_sent = False

    def state_cb(self, msg):
        self.current_state = msg

    def pos_cb(self, msg: PoseStamped):
        self.current_pose[0] = msg.pose.position.x
        self.current_pose[1] = msg.pose.position.y
        self.current_pose[2] = msg.pose.position.z

    def cmdloop_callback(self):
        # Generate circular trajectory
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.radius * math.cos(self.theta)
        pose.pose.position.y = self.radius * math.sin(self.theta)
        pose.pose.position.z = self.current_pose[2]

        # Publish setpoint
        self.local_pos_pub.publish(pose)
        self.sent_setpoints += 1

        # Send a few setpoints before switching to OFFBOARD
        if self.sent_setpoints == 100:
            self.set_mode("OFFBOARD")
            self.arm()

        # Increment angle
        self.theta += self.omega * self.timer_period

    def arm(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn("Arming service not ready")
            return
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)
        self.get_logger().info("Arming request sent")

    def set_mode(self, mode):
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn("SetMode service not ready")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
        self.get_logger().info(f"Mode change request: {mode}")


def main(args=None):
    rclpy.init(args=args)
    node = MavrosOffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
