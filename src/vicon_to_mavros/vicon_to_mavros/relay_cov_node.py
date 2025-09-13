#!/usr/bin/env python3
"""
ROS2 Vicon -> MAVROS relay (improved)

- Subscribes to TransformStamped from vicon_bridge
- Converts/optionally converts ENU->NED
- Publishes geometry_msgs/PoseWithCovarianceStamped to /mavros/vision_pose/pose

Make sure /mavros is running and vision_pose plugin is enabled.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import math
import numpy as np

class ViconMavrosRelayNode(Node):
    def __init__(self):
        super().__init__('vicon_mavros_relay')

        # Parameters (same names you used)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('vicon_subject_name', 'whiskerdrone'),
                ('vicon_segment_name', 'whiskerdrone'),
                ('vicon_tf_namespace', 'vicon'),
                ('world_frame_id', 'map'),
                ('base_link_frame_id', 'base_link'),
                ('publish_tf', True),
                ('mavros_vision_pose_topic', '/mavros/vision_pose/pose_cov'),
                ('invert_z_axis', False),
                ('apply_enu_to_ned', False),
                ('position_offset_x', 0.0),
                ('position_offset_y', 0.0),
                ('position_offset_z', 0.0),
                ('publish_rate_hz', 60.0),
                ('position_variance', 0.01),
                ('orientation_variance', 0.01),
            ]
        )

        # get params
        p = lambda n: self.get_parameter(n).value
        self.vicon_subject_name = p('vicon_subject_name')
        self.vicon_segment_name = p('vicon_segment_name')
        self.vicon_tf_namespace = p('vicon_tf_namespace')
        self.world_frame_id = p('world_frame_id')
        self.base_link_frame_id = p('base_link_frame_id')
        self.publish_tf = p('publish_tf')
        self.mavros_topic = p('mavros_vision_pose_topic')
        self.invert_z = p('invert_z_axis')
        self.apply_enu_to_ned = p('apply_enu_to_ned')
        self.pos_offset_x = p('position_offset_x')
        self.pos_offset_y = p('position_offset_y')
        self.pos_offset_z = p('position_offset_z')
        self.publish_rate = float(p('publish_rate_hz'))
        self.pos_var = float(p('position_variance'))
        self.ori_var = float(p('orientation_variance'))

        # build topic name
        self.vicon_topic = f"/{self.vicon_tf_namespace}/{self.vicon_subject_name}/{self.vicon_segment_name}"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # publisher: PoseWithCovarianceStamped (better for FCU)
        self.mavros_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.mavros_topic,
            qos_profile
        )

        # subscriber: TransformStamped from vicon_bridge
        self.vicon_sub = self.create_subscription(
            TransformStamped,
            self.vicon_topic,
            self.vicon_callback,
            qos_profile
        )

        # rate limiter
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.latest_msg = None

        self.get_logger().info(f"Started vicon->mavros relay")
        self.get_logger().info(f"Subscribing to: {self.vicon_topic}")
        self.get_logger().info(f"Publishing to: {self.mavros_topic} (PoseWithCovarianceStamped)")
        self.get_logger().info(f"apply_enu_to_ned={self.apply_enu_to_ned}, invert_z={self.invert_z}")

    def enu_to_ned(self, x, y, z):
        # Conservative mapping consistent with many MAVROS utilities:
        # ENU (x_east, y_north, z_up) -> NED (x_north, y_east, z_down)
        # So result = (y, x, -z)
        return y, x, -z

    def vicon_callback(self, msg: TransformStamped):
        """
        Accept TransformStamped from vicon_bridge and convert to PoseWithCovarianceStamped
        """
        out = PoseWithCovarianceStamped()

        # forward original timestamp if available, else use now()
        if msg.header and msg.header.stamp.sec != 0:
            out.header.stamp = msg.header.stamp
        else:
            out.header.stamp = self.get_clock().now().to_msg()

        # this frame is what MAVROS plugin expects (default: "map"), make configurable
        out.header.frame_id = self.world_frame_id

        # read translation and rotation
        tx = msg.transform.translation.x + self.pos_offset_x
        ty = msg.transform.translation.y + self.pos_offset_y
        tz = msg.transform.translation.z + self.pos_offset_z

        q = msg.transform.rotation  # geometry_msgs/Quaternion

        # optional inversion of z-axis (keep for backward compatibility)
        if self.invert_z:
            tz = -tz

        # apply ENU->NED if requested (this affects both pos & orientation!)
        if self.apply_enu_to_ned:
            # position
            nx, ny, nz = self.enu_to_ned(tx, ty, tz)
            out.pose.pose.position.x = nx
            out.pose.pose.position.y = ny
            out.pose.pose.position.z = nz

            # orientation: convert quaternion ENU->NED by swapping axes and flipping z sign in quaternion space:
            # A safe/simple approach: convert quaternion to (x,y,z,w) then apply axis permutation
            # For common mapping ENU->NED, transform q = (qx, qy, qz, qw) -> q' = (qy, qx, -qz, qw)
            out.pose.pose.orientation.x = q.y
            out.pose.pose.orientation.y = q.x
            out.pose.pose.orientation.z = -q.z
            out.pose.pose.orientation.w = q.w
        else:
            out.pose.pose.position.x = tx
            out.pose.pose.position.y = ty
            out.pose.pose.position.z = tz
            out.pose.pose.orientation = q

        # Fill covariance: diag = [pos_var,pos_var,pos_var,ori_var,ori_var,ori_var]
        cov = [0.0] * 36
        cov[0] = self.pos_var
        cov[7] = self.pos_var
        cov[14] = self.pos_var
        cov[21] = self.ori_var
        cov[28] = self.ori_var
        cov[35] = self.ori_var
        out.pose.covariance = cov

        # stash for timed publish
        self.latest_msg = out

        self.get_logger().debug(
            f"Received Vicon: tx={tx:.3f} ty={ty:.3f} tz={tz:.3f} -> published pos={out.pose.pose.position.x:.3f},"
            f"{out.pose.pose.position.y:.3f},{out.pose.pose.position.z:.3f}"
        )

    def timer_callback(self):
        if self.latest_msg is None:
            return
        # update stamp to now() to reflect freshness for PX4 (still keep original stamp in header if you prefer)
        # we keep header.stamp from vicon but also refresh now() optionally: here we leave original stamp.
        self.mavros_pub.publish(self.latest_msg)
        self.get_logger().debug("Published PoseWithCovarianceStamped to MAVROS")

def main(args=None):
    rclpy.init(args=args)
    node = ViconMavrosRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
