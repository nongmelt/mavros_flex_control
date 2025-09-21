#!/usr/bin/env python3

import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from flex_sensor_reader.msg import FlexSensorData

# MAVROS messages and services
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, ParamSet
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header, Float32
from scipy.spatial.transform import Rotation as R
from simple_pid import PID


class FlexOffboardControl(Node):

    def __init__(self):
        super().__init__("flex_mavros_offboard_control")

        # Declare and retrieve parameters
        self.declare_parameter("max_pitch_angle", 15.0)  # Max pitch angle in degrees
        self.declare_parameter("max_yaw_rate", 30.0)  # Max yaw rate in deg/s
        self.declare_parameter("deadzone_threshold", 100.0)
        self.declare_parameter("enable_pitch_control", True)
        self.declare_parameter("enable_yaw_control", True)
        self.declare_parameter("pitch_kp", 0.00015)
        self.declare_parameter("pitch_kd", 0.001)
        self.declare_parameter("yaw_kp", 0.05)
        self.declare_parameter("yaw_kd", 0.001)
        self.declare_parameter("pitch_setpoint", 2000.0)
        self.declare_parameter("yaw_setpoint", 0.0)

        self.declare_parameter(
            "max_velocity_x", 2.0
        )  # Max forward/backward velocity in m/s
        self.declare_parameter("max_velocity_y", 2.0)  # Max left/right velocity in m/s
        self.declare_parameter("max_velocity_z", 1.0)  # Max up/down velocity in m/s

        # PID controller parameters
        pitch_kp = self.get_parameter("pitch_kp").get_parameter_value().double_value
        pitch_kd = self.get_parameter("pitch_kd").get_parameter_value().double_value
        self.pitch_setpoint = (
            self.get_parameter("pitch_setpoint").get_parameter_value().double_value
        )
        self.pitch_pid = PID(pitch_kp, 0, pitch_kd, setpoint=self.pitch_setpoint)

        yaw_kp = self.get_parameter("yaw_kp").get_parameter_value().double_value
        yaw_kd = self.get_parameter("yaw_kd").get_parameter_value().double_value
        self.yaw_setpoint = (
            self.get_parameter("yaw_setpoint").get_parameter_value().double_value
        )
        self.yaw_pid = PID(yaw_kp, 0, yaw_kd, setpoint=self.yaw_setpoint)

        # Set PID output limits
        self.pitch_pid.output_limits = (
            -self.get_parameter("max_pitch_angle").get_parameter_value().double_value,
            self.get_parameter("max_pitch_angle").get_parameter_value().double_value,
        )
        self.yaw_pid.output_limits = (
            -self.get_parameter("max_yaw_rate").get_parameter_value().double_value,
            self.get_parameter("max_yaw_rate").get_parameter_value().double_value,
        )

        # Control mode settings
        self.max_velocity_x = (
            self.get_parameter("max_velocity_x").get_parameter_value().double_value
        )
        self.max_velocity_y = (
            self.get_parameter("max_velocity_y").get_parameter_value().double_value
        )
        self.max_velocity_z = (
            self.get_parameter("max_velocity_z").get_parameter_value().double_value
        )

        # Flex sensor parameters
        self.max_pitch_angle = (
            self.get_parameter("max_pitch_angle").get_parameter_value().double_value
        )
        self.max_yaw_rate = (
            self.get_parameter("max_yaw_rate").get_parameter_value().double_value
        )
        self.deadzone_threshold = (
            self.get_parameter("deadzone_threshold").get_parameter_value().double_value
        )
        self.enable_pitch_control = (
            self.get_parameter("enable_pitch_control").get_parameter_value().bool_value
        )
        self.enable_yaw_control = (
            self.get_parameter("enable_yaw_control").get_parameter_value().bool_value
        )

        # State variables
        self.dt = 0.02  # 50 Hz
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.offboard_enabled = False
        self.armed = False

        # Flex sensor control variables
        self.flex_channel_1 = 0.0  # First flex sensor channel
        self.flex_channel_2 = 0.0  # Second flex sensor channel
        self.flex_pitch_input = 0.0  # Sum of channels 1 and 2 for pitch
        self.flex_yaw_input = 0.0  # Difference of channels 1 and 2 for yaw

        # Control outputs - used for both attitude and velocity control
        self.current_pitch_rate = (
            0.0  # Target pitch angle in degrees (attitude) or velocity_x (velocity)
        )
        self.current_pitch_angle = 0.0
        self.current_yaw_rate = 0.0  # Target yaw rate in deg/s
        self.current_yaw_angle = 0.0  # Accumulated yaw angle

        # Velocity control outputs
        self.target_velocity_x = 0.0  # Forward/backward velocity in m/s
        self.target_velocity_y = 0.0  # Left/right velocity in m/s
        self.target_velocity_z = 0.0  # Up/down velocity in m/s

        # QoS profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # MAVROS Subscribers
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.vehicle_status_callback,
            qos_profile_sub,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_profile_sub,
        )

        # Flex sensor subscriber
        self.flex_sensor_sub = self.create_subscription(
            FlexSensorData,
            "/flex_sensors/processed",
            self.flex_sensor_callback,
            qos_profile_sub,
        )

        # MAVROS Publishers
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        # Debug publishers
        self.pitch_cmd_publisher = self.create_publisher(
            Float32, "flex_control/pitch_angle", qos_profile_pub
        )
        self.yaw_cmd_publisher = self.create_publisher(
            Float32, "flex_control/yaw_rate", qos_profile_pub
        )
        self.flex_pitch_pub = self.create_publisher(
            Float32, "flex_control/flex_pitch_input", qos_profile_pub
        )
        self.flex_yaw_pub = self.create_publisher(
            Float32, "flex_control/flex_yaw_input", qos_profile_pub
        )
        # Velocity debug publishers
        self.vel_x_pub = self.create_publisher(
            Float32, "flex_control/velocity_x", qos_profile_pub
        )
        self.vel_y_pub = self.create_publisher(
            Float32, "flex_control/velocity_y", qos_profile_pub
        )

        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Wait for services
        self.get_logger().info("Waiting for MAVROS services...")
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("MAVROS services available")

        # Wait for pose data
        self.get_logger().info("Waiting for pose data...")
        pose_timeout = 0
        while pose_timeout < 50:
            if (
                abs(self.current_pose.pose.position.x) > 0.01
                or abs(self.current_pose.pose.position.z) > 0.01
            ):
                self.get_logger().info("Pose data received!")
                break
            pose_timeout += 1
            rclpy.spin_once(self, timeout_sec=0.1)

        if pose_timeout >= 50:
            self.get_logger().warn(
                "No pose data received - check PX4 EKF2 and MAVROS configuration"
            )

        # Main control timer
        timer_period = self.dt  # 50Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        # Counter for initial setpoints
        self.setpoint_counter = 0
        self.last_mode_request = self.get_clock().now()
        self.mode_request_timeout = 2.0

        self.get_logger().info("Flex Offboard Control initialized with:")
        self.get_logger().info(f"  - Max pitch angle: {self.max_pitch_angle}°")
        self.get_logger().info(f"  - Max yaw rate: {self.max_yaw_rate}°/s")
        self.get_logger().info(f"  - Control frequency: {1.0/self.dt:.1f}Hz")
        self.get_logger().info(f"  - Pitch PID: Kp={pitch_kp}, Kd={pitch_kd}")
        self.get_logger().info(f"  - Yaw PID: Kp={yaw_kp}, Kd={yaw_kd}")
        self.get_logger().info(
            f"  - Max velocities: X={self.max_velocity_x}m/s, Y={self.max_velocity_y}m/s, Z={self.max_velocity_z}m/s"
        )

    def apply_deadzone(self, error, deadzone_size):
        if abs(error) < deadzone_size:
            return 0.0
        elif error > deadzone_size:
            return error - deadzone_size
        else:
            return error + deadzone_size

    def flex_sensor_callback(self, msg):
        """Process flex sensor data from Float32MultiArray"""
        if len(msg.resistance_values) >= 4:
            # Extract the 4 channels
            channel_0 = msg.resistance_values[0]  # Not used in this implementation
            self.flex_channel_1 = msg.resistance_values[
                1
            ]  # Channel 1 for pitch/yaw calculation
            self.flex_channel_2 = msg.resistance_values[
                2
            ]  # Channel 2 for pitch/yaw calculation
            channel_3 = msg.resistance_values[3]  # Not used in this implementation

            # Calculate pitch input (sum of channels 1 and 2)
            self.flex_pitch_input = self.flex_channel_1 + self.flex_channel_2

            # Calculate yaw input (difference of channels 1 and 2)
            self.flex_yaw_input = self.flex_channel_1 - self.flex_channel_2

            # Apply deadzone to reduce noise
            self.flex_pitch_input = self.apply_deadzone(
                self.flex_pitch_input, self.deadzone_threshold
            )
            self.flex_yaw_input = self.apply_deadzone(
                self.flex_yaw_input, self.deadzone_threshold
            )

            # Publish debug information
            pitch_debug_msg = Float32()
            pitch_debug_msg.data = self.flex_pitch_input
            self.flex_pitch_pub.publish(pitch_debug_msg)

            yaw_debug_msg = Float32()
            yaw_debug_msg.data = self.flex_yaw_input
            self.flex_yaw_pub.publish(yaw_debug_msg)
        else:
            self.get_logger().warn(
                f"Received flex sensor data with {len(msg.data)} channels, expected at least 4"
            )

    def update_control_commands(self):
        """Update pitch and yaw commands using PD controllers"""
        # Velocity control mode
        if self.enable_pitch_control:
            velocity_command = self.pitch_pid(self.flex_pitch_input)
            self.target_velocity_x = np.clip(
                velocity_command, -self.max_velocity_x, self.max_velocity_x
            )
        else:
            self.target_velocity_x = 0.0

        if self.enable_yaw_control:
            # Still calculate yaw rate for rotational control
            self.current_yaw_rate = self.yaw_pid(self.flex_yaw_input)
            self.current_yaw_rate = np.clip(
                self.current_yaw_rate, -self.max_yaw_rate, self.max_yaw_rate
            )
        else:
            self.target_velocity_y = 0.0
            self.current_yaw_rate = 0.0

        # Maintain altitude (no vertical velocity)
        self.target_velocity_z = 0.0

    def vehicle_status_callback(self, msg):
        """Callback for MAVROS state updates"""
        prev_state = self.current_state
        self.current_state = msg

        self.offboard_enabled = msg.mode == "OFFBOARD"
        self.armed = msg.armed

        # Log important state changes
        if prev_state.mode != msg.mode:
            self.get_logger().info(f"Mode changed: {prev_state.mode} → {msg.mode}")

        if prev_state.armed != self.armed:
            self.get_logger().info(f'Vehicle: {"ARMED" if self.armed else "DISARMED"}')

    def pose_callback(self, msg):
        """Callback for current pose updates"""
        self.current_pose = msg

    def set_offboard_mode(self):
        """Switch to offboard mode"""
        current_time = self.get_clock().now()
        if (
            current_time.nanoseconds - self.last_mode_request.nanoseconds
        ) / 1e9 < self.mode_request_timeout:
            return None

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.last_mode_request = current_time
        future = self.set_mode_client.call_async(req)
        return future

    def publish_velocity_setpoint(self, vx, vy, vz, yaw_rate):
        """Publish velocity setpoint"""
        twist = TwistStamped()
        twist.header = Header()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"

        # Linear velocity (ENU frame)
        twist.twist.linear.x = vx  # Forward/backward
        twist.twist.linear.y = vy  # Left/right
        twist.twist.linear.z = vz  # Up/down

        # Angular velocity
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = math.radians(yaw_rate)  # Convert deg/s to rad/s

        self.velocity_pub.publish(twist)

    def cmdloop_callback(self):
        """Main control loop"""

        if not self.current_state.armed:
            return
        self.update_flight_state()
        # Use velocity control for flex sensor input
        self.update_control_commands()
        self.publish_velocity_setpoint(
            self.target_velocity_x,  # Forward/backward from pitch input
            self.target_velocity_y,  # Left/right from yaw input
            self.target_velocity_z,  # Up/down (usually 0 for altitude hold)
            self.current_yaw_rate,  # Yaw rotation
        )
        if self.setpoint_counter < 100:
            self.setpoint_counter += 1
            if self.setpoint_counter == 50:
                self.get_logger().info(
                    "Sending initial setpoints before enabling offboard..."
                )
            return

        # Flight state machine
        self.setpoint_counter += 1

    def update_flight_state(self):
        """Update flight state machine"""

        if self.current_state.connected:
            if not self.offboard_enabled:
                if self.current_state.mode not in ["AUTO.RTL", "AUTO.LAND"]:
                    self.get_logger().warn(
                        f"Switching back to OFFBOARD from {self.current_state.mode}"
                    )
                    self.set_offboard_mode()

        if self.setpoint_counter % 250 == 0:  # Every 5 seconds
            current_alt = self.current_pose.pose.position.z
            self.get_logger().info(
                f"🎮 Velocity Control: Pitch_in={self.flex_pitch_input:.1f}, "
                f"Yaw_in={self.flex_yaw_input:.1f}, Vel_X={self.target_velocity_x:.2f}m/s, "
                f"Vel_Y={self.target_velocity_y:.2f}m/s, YawRate={self.current_yaw_rate:.1f}°/s, "
                f"Alt={current_alt:.2f}m"
            )


def main(args=None):
    rclpy.init(args=args)
    flex_offboard_control = FlexOffboardControl()

    try:
        rclpy.spin(flex_offboard_control)
    except KeyboardInterrupt:
        flex_offboard_control.get_logger().info("Shutting down...")
    finally:
        flex_offboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
