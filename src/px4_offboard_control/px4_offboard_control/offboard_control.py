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

# MAVROS messages and services
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, ParamSet
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header, Float32MultiArray, Float32
from scipy.spatial.transform import Rotation as R
from simple_pid import PID


class FlexOffboardControl(Node):

    def __init__(self):
        super().__init__("flex_mavros_offboard_control")

        # Declare and retrieve parameters
        self.declare_parameter("namespace", "")
        self.declare_parameter("altitude", 1.5)
        self.declare_parameter("takeoff_speed", 1.0)
        self.declare_parameter("hover_time", 2.0)
        self.declare_parameter("normal_scale", 1.0)
        self.declare_parameter("balance_scale", 1.0)
        self.declare_parameter("max_pitch_angle", 15.0)  # Max pitch angle in degrees
        self.declare_parameter("max_yaw_rate", 30.0)  # Max yaw rate in deg/s
        self.declare_parameter("deadzone_threshold", 100.0)
        self.declare_parameter("enable_flex_control", True)
        self.declare_parameter("enable_pitch_control", True)
        self.declare_parameter("enable_yaw_control", True)
        self.declare_parameter("hover_thrust", 0.5)
        self.declare_parameter("pitch_kp", 0.00015)
        self.declare_parameter("pitch_kd", 0.001)
        self.declare_parameter("yaw_kp", 0.05)
        self.declare_parameter("yaw_kd", 0.001)
        self.declare_parameter("pitch_setpoint", 2000.0)
        self.declare_parameter("yaw_setpoint", 0.0)
        
        # Control mode parameters
        self.declare_parameter("use_velocity_control", False)  # Toggle between attitude and velocity control
        self.declare_parameter("max_velocity_x", 2.0)  # Max forward/backward velocity in m/s
        self.declare_parameter("max_velocity_y", 2.0)  # Max left/right velocity in m/s
        self.declare_parameter("max_velocity_z", 1.0)  # Max up/down velocity in m/s

        # PID controller parameters
        pitch_kp = self.get_parameter("pitch_kp").value
        pitch_kd = self.get_parameter("pitch_kd").value
        self.pitch_setpoint = self.get_parameter("pitch_setpoint").value
        self.pitch_pid = PID(pitch_kp, 0, pitch_kd, setpoint=self.pitch_setpoint)
        
        yaw_kp = self.get_parameter("yaw_kp").value
        yaw_kd = self.get_parameter("yaw_kd").value
        self.yaw_setpoint = self.get_parameter("yaw_setpoint").value
        # Fixed typo: was yaw_self.setpoint, now self.yaw_setpoint
        self.yaw_pid = PID(yaw_kp, 0, yaw_kd, setpoint=self.yaw_setpoint)

        # Set PID output limits
        self.pitch_pid.output_limits = (-self.get_parameter("max_pitch_angle").value, 
                                       self.get_parameter("max_pitch_angle").value)
        self.yaw_pid.output_limits = (-self.get_parameter("max_yaw_rate").value, 
                                     self.get_parameter("max_yaw_rate").value)

        # Control mode settings
        self.use_velocity_control = self.get_parameter("use_velocity_control").value
        self.max_velocity_x = self.get_parameter("max_velocity_x").value
        self.max_velocity_y = self.get_parameter("max_velocity_y").value
        self.max_velocity_z = self.get_parameter("max_velocity_z").value

        self.namespace = self.get_parameter("namespace").value
        self.namespace_prefix = f"/{self.namespace}" if self.namespace else ""

        # Parameters
        self.altitude = self.get_parameter("altitude").value
        self.takeoff_speed = self.get_parameter("takeoff_speed").value
        hover_time = self.get_parameter("hover_time").value

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
        self.enable_flex_control = (
            self.get_parameter("enable_flex_control").get_parameter_value().bool_value
        )
        self.enable_pitch_control = (
            self.get_parameter("enable_pitch_control").get_parameter_value().bool_value
        )
        self.enable_yaw_control = (
            self.get_parameter("enable_yaw_control").get_parameter_value().bool_value
        )
        self.hover_thrust = (
            self.get_parameter("hover_thrust").get_parameter_value().double_value
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
        self.flex_yaw_input = 0.0   # Difference of channels 1 and 2 for yaw
        
        # Control outputs - used for both attitude and velocity control
        self.current_pitch_rate = 0.0  # Target pitch angle in degrees (attitude) or velocity_x (velocity)
        self.current_pitch_angle = 0.0
        self.current_yaw_rate = 0.0  # Target yaw rate in deg/s
        self.current_yaw_angle = 0.0  # Accumulated yaw angle
        
        # Velocity control outputs
        self.target_velocity_x = 0.0  # Forward/backward velocity in m/s
        self.target_velocity_y = 0.0  # Left/right velocity in m/s
        self.target_velocity_z = 0.0  # Up/down velocity in m/s

        # Flight state machine
        self.flight_state = "FLEX_CONTROL"
        self.takeoff_complete = False
        self.hover_timer = 0
        self.hover_duration = int(hover_time / self.dt)

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
            f"{self.namespace_prefix}/mavros/state",
            self.vehicle_status_callback,
            qos_profile_sub,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            f"{self.namespace_prefix}/mavros/local_position/pose",
            self.pose_callback,
            qos_profile_sub,
        )

        # Flex sensor subscriber
        self.flex_sensor_sub = self.create_subscription(
            Float32MultiArray,
            "/flex_sensors/processed",
            self.flex_sensor_callback,
            qos_profile_sub,
        )

        # MAVROS Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            f"{self.namespace_prefix}/mavros/setpoint_position/local",
            qos_profile_pub,
        )

        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            f"{self.namespace_prefix}/mavros/setpoint_raw/attitude",
            qos_profile_pub,
        )

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            f"{self.namespace_prefix}/mavros/setpoint_velocity/cmd_vel",
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

        # MAVROS Service clients
        self.arming_client = self.create_client(
            CommandBool, f"{self.namespace_prefix}/mavros/cmd/arming"
        )
        self.set_mode_client = self.create_client(
            SetMode, f"{self.namespace_prefix}/mavros/set_mode"
        )
        
        self.param_get_client = self.create_client(
            ParamGet, f"{self.namespace_prefix}/mavros/param/get"
        )
        self.param_set_client = self.create_client(
            ParamSet, f"{self.namespace_prefix}/mavros/param/set"
        )
        
        # Store original I gains to restore later
        self.original_i_gains = {}
        self.i_gains_modified = False
        
        # Rate controller I gain parameter names for PX4
        self.rate_i_params = [
            "MC_ROLLRATE_I",   # Roll rate I gain
            "MC_PITCHRATE_I",  # Pitch rate I gain
            "MC_YAWRATE_I"     # Yaw rate I gain
        ]

        # Wait for services
        self.get_logger().info("Waiting for MAVROS services...")
        self.arming_client.wait_for_service(timeout_sec=10.0)
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
        self.get_logger().info(f"  - Takeoff altitude: {self.altitude}m")
        self.get_logger().info(f"  - Max pitch angle: {self.max_pitch_angle}°")
        self.get_logger().info(f"  - Max yaw rate: {self.max_yaw_rate}°/s")
        self.get_logger().info(f"  - Control frequency: {1.0/self.dt:.1f}Hz")
        self.get_logger().info(f"  - Pitch PID: Kp={pitch_kp}, Kd={pitch_kd}")
        self.get_logger().info(f"  - Yaw PID: Kp={yaw_kp}, Kd={yaw_kd}")
        self.get_logger().info(f"  - Control mode: {'Velocity' if self.use_velocity_control else 'Attitude'}")
        if self.use_velocity_control:
            self.get_logger().info(f"  - Max velocities: X={self.max_velocity_x}m/s, Y={self.max_velocity_y}m/s, Z={self.max_velocity_z}m/s")
    
    def apply_deadzone(self, error, deadzone_size):
        if (abs(error) < deadzone_size):
            return 0.0
        elif (error > deadzone_size):
            return error - deadzone_size
        else:
            return error + deadzone_size
        
    def get_px4_parameter(self, param_name):
        """Get a PX4 parameter value"""
        if not self.param_get_client.service_is_ready():
            self.get_logger().warn("Parameter get service not ready")
            return None
            
        req = ParamGet.Request()
        req.param_id = param_name
        
        try:
            future = self.param_get_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    if response.value.integer != 0:
                        return float(response.value.integer)
                    else:
                        return response.value.real
                else:
                    self.get_logger().warn(f"Failed to get parameter {param_name}")
                    return None
            else:
                self.get_logger().warn(f"Timeout getting parameter {param_name}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error getting parameter {param_name}: {str(e)}")
            return None
    
    def set_px4_parameter(self, param_name, value):
        """Set a PX4 parameter value"""
        if not self.param_set_client.service_is_ready():
            self.get_logger().warn("Parameter set service not ready")
            return False
            
        req = ParamSet.Request()
        req.param_id = param_name
        
        # Set parameter value (PX4 parameters are typically float)
        req.value.integer = 0
        req.value.real = float(value)
        
        try:
            future = self.param_set_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Successfully set {param_name} = {value}")
                    return True
                else:
                    self.get_logger().warn(f"Failed to set parameter {param_name} = {value}")
                    return False
            else:
                self.get_logger().warn(f"Timeout setting parameter {param_name}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error setting parameter {param_name}: {str(e)}")
            return False
    
    def backup_and_disable_rate_i_gains(self):
        """Backup original I gains and set them to zero"""
        if self.i_gains_modified:
            self.get_logger().info("Rate I gains already modified")
            return True
            
        self.get_logger().info("Backing up and disabling rate controller I gains...")
        
        success = True
        for param_name in self.rate_i_params:
            # Get original value
            original_value = self.get_px4_parameter(param_name)
            if original_value is not None:
                self.original_i_gains[param_name] = original_value
                self.get_logger().info(f"Backed up {param_name}: {original_value}")
                
                # Set to zero
                if self.set_px4_parameter(param_name, 0.0):
                    self.get_logger().info(f"Disabled {param_name} (set to 0.0)")
                else:
                    success = False
            else:
                self.get_logger().error(f"Failed to backup {param_name}")
                success = False
        
        if success:
            self.i_gains_modified = True
            self.get_logger().info("✅ All rate controller I gains disabled for flex control")
        else:
            self.get_logger().error("❌ Failed to disable some rate controller I gains")
            
        return success
    
    def restore_rate_i_gains(self):
        """Restore original I gains"""
        if not self.i_gains_modified:
            self.get_logger().info("Rate I gains not modified, nothing to restore")
            return True
            
        self.get_logger().info("Restoring original rate controller I gains...")
        
        success = True
        for param_name, original_value in self.original_i_gains.items():
            if self.set_px4_parameter(param_name, original_value):
                self.get_logger().info(f"Restored {param_name}: {original_value}")
            else:
                success = False
        
        if success:
            self.i_gains_modified = False
            self.get_logger().info("✅ All rate controller I gains restored")
        else:
            self.get_logger().error("❌ Failed to restore some rate controller I gains")
            
        return success

    def flex_sensor_callback(self, msg):
        """Process flex sensor data from Float32MultiArray"""
        if len(msg.data) >= 4:
            # Extract the 4 channels
            channel_0 = msg.data[0]  # Not used in this implementation
            self.flex_channel_1 = msg.data[1]  # Channel 1 for pitch/yaw calculation
            self.flex_channel_2 = msg.data[2]  # Channel 2 for pitch/yaw calculation
            channel_3 = msg.data[3]  # Not used in this implementation
            
            # Calculate pitch input (sum of channels 1 and 2)
            self.flex_pitch_input = self.flex_channel_1 + self.flex_channel_2
            
            # Calculate yaw input (difference of channels 1 and 2)
            self.flex_yaw_input = self.flex_channel_1 - self.flex_channel_2
            
            # Apply deadzone to reduce noise
            self.flex_pitch_input = self.apply_deadzone(self.flex_pitch_input, self.deadzone_threshold)
            self.flex_yaw_input = self.apply_deadzone(self.flex_yaw_input, self.deadzone_threshold)
            
            # Publish debug information
            pitch_debug_msg = Float32()
            pitch_debug_msg.data = self.flex_pitch_input
            self.flex_pitch_pub.publish(pitch_debug_msg)
            
            yaw_debug_msg = Float32()
            yaw_debug_msg.data = self.flex_yaw_input
            self.flex_yaw_pub.publish(yaw_debug_msg)
        else:
            self.get_logger().warn(f"Received flex sensor data with {len(msg.data)} channels, expected at least 4")

    def update_control_commands(self):
        """Update pitch and yaw commands using PD controllers"""
        if self.use_velocity_control:
            # Velocity control mode
            if self.enable_pitch_control:
                # Map flex input to forward/backward velocity
                # Normalize PID output for velocity control
                velocity_command = self.pitch_pid(self.flex_pitch_input)
                # Scale to velocity range
                # velocity_scale = self.max_velocity_x / max(abs(self.max_pitch_angle), 1.0)
                self.target_velocity_x = np.clip(
                    velocity_command,
                    -self.max_velocity_x,
                    self.max_velocity_x
                )
            else:
                self.target_velocity_x = 0.0
            
            if self.enable_yaw_control:
                # Map flex input to left/right velocity  
                # velocity_command = self.yaw_pid(self.flex_yaw_input)
                # # Scale to velocity range
                # velocity_scale = self.max_velocity_y / max(abs(self.max_yaw_rate), 1.0)
                # self.target_velocity_y = np.clip(
                #     velocity_command * velocity_scale,
                #     -self.max_velocity_y,
                #     self.max_velocity_y
                # )
                
                # Still calculate yaw rate for rotational control
                self.current_yaw_rate = self.yaw_pid(self.flex_yaw_input)
                self.current_yaw_rate = np.clip(
                    self.current_yaw_rate,
                    -self.max_yaw_rate,
                    self.max_yaw_rate
                )
            else:
                self.target_velocity_y = 0.0
                self.current_yaw_rate = 0.0
            
            # Maintain altitude (no vertical velocity)
            self.target_velocity_z = 0.0
            
        else:
            # Attitude control mode (original behavior)
            if self.enable_pitch_control:
                # Use PID controller for pitch (with I=0, so it's effectively PD)
                self.current_pitch_angle = self.pitch_pid(self.flex_pitch_input)
                
                # Clamp to maximum angle
                self.current_pitch_angle = np.clip(
                    self.current_pitch_angle, 
                    -self.max_pitch_angle, 
                    self.max_pitch_angle
                )
            else:
                self.current_pitch_angle = 0.0
            
            # Yaw rate control (PD controller)
            if self.enable_yaw_control:
                # Use PID controller for yaw rate (with I=0, so it's effectively PD)
                self.current_yaw_rate = self.yaw_pid(self.flex_yaw_input)
                
                # Clamp to maximum rate
                self.current_yaw_rate = np.clip(
                    self.current_yaw_rate,
                    -self.max_yaw_rate,
                    self.max_yaw_rate
                )
                
                # Integrate yaw rate to get yaw angle for attitude control
                self.current_yaw_angle += self.current_yaw_rate * self.dt
                
                # Normalize yaw angle to [-180, 180]
                while self.current_yaw_angle > 180.0:
                    self.current_yaw_angle -= 360.0
                while self.current_yaw_angle < -180.0:
                    self.current_yaw_angle += 360.0
            else:
                self.current_yaw_rate = 0.0

    def publish_attitude(self):
        """Publish attitude setpoint with proper thrust"""
        att_msg = AttitudeTarget()
        att_msg.header.stamp = self.get_clock().now().to_msg()
        att_msg.header.frame_id = "map"

        # Convert angles to radians
        pitch_rad = math.radians(self.current_pitch_angle)
        yaw_rad = math.radians(self.current_yaw_angle)
        roll_rad = 0.0

        r = R.from_euler("xyz", [roll_rad, pitch_rad, yaw_rad], degrees=False)
        q = r.as_quat(canonical=True)

        att_msg.orientation.x = q[0]
        att_msg.orientation.y = q[1]
        att_msg.orientation.z = q[2]
        att_msg.orientation.w = q[3]

        att_msg.thrust = self.hover_thrust

        # Set type mask to use orientation only (ignore body rates)
        att_msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )

        self.attitude_pub.publish(att_msg)

        # Publish debug info
        if self.use_velocity_control:
            # In velocity mode, publish velocity commands
            pitch_msg = Float32()
            pitch_msg.data = self.target_velocity_x
            self.pitch_cmd_publisher.publish(pitch_msg)
            
            vel_x_msg = Float32()
            vel_x_msg.data = self.target_velocity_x
            self.vel_x_pub.publish(vel_x_msg)
            
            vel_y_msg = Float32()
            vel_y_msg.data = self.target_velocity_y
            self.vel_y_pub.publish(vel_y_msg)
        else:
            # In attitude mode, publish attitude commands
            pitch_msg = Float32()
            pitch_msg.data = self.current_pitch_angle
            self.pitch_cmd_publisher.publish(pitch_msg)

        yaw_msg = Float32()
        yaw_msg.data = self.current_yaw_rate
        self.yaw_cmd_publisher.publish(yaw_msg)

    def vehicle_status_callback(self, msg):
        """Callback for MAVROS state updates"""
        prev_state = self.current_state
        self.current_state = msg

        self.offboard_enabled = msg.mode == "OFFBOARD"
        self.armed = msg.armed

        # Log important state changes
        if prev_state.mode != msg.mode:
            if msg.mode == "AUTO.RTL":
                self.get_logger().error("⚠️  FAILSAFE TRIGGERED: Mode changed to RTL")
            else:
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

    def arm_vehicle(self):
        """Arm the vehicle"""
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        return future

    def is_at_altitude(self, target_altitude, tolerance=0.3):
        """Check if vehicle has reached target altitude"""
        current_altitude = self.current_pose.pose.position.z
        return abs(current_altitude - target_altitude) < tolerance

    def get_takeoff_setpoint(self):
        """Get takeoff setpoint"""
        return (0.0, 0.0, self.altitude)

    def get_hover_setpoint(self):
        """Get hover setpoint"""
        return (0.0, 0.0, self.altitude)

    def publish_position_setpoint(self, x, y, z, yaw=0.0):
        """Publish position setpoint"""
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Quaternion from yaw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.local_pos_pub.publish(pose)

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

        # CRITICAL: Publish setpoints at high frequency
        if self.flight_state in ["INIT", "TAKEOFF"]:
            x_setpoint, y_setpoint, z_setpoint = self.get_takeoff_setpoint()
            self.publish_position_setpoint(x_setpoint, y_setpoint, z_setpoint)
        elif self.flight_state == "HOVER":
            x_setpoint, y_setpoint, z_setpoint = self.get_hover_setpoint()
            self.publish_position_setpoint(x_setpoint, y_setpoint, z_setpoint)
        elif self.flight_state == "FLEX_CONTROL":
            if self.enable_flex_control:
                if self.use_velocity_control:
                    # Use velocity control for flex sensor input
                    self.update_control_commands()
                    self.publish_velocity_setpoint(
                        self.target_velocity_x,  # Forward/backward from pitch input
                        self.target_velocity_y,  # Left/right from yaw input  
                        self.target_velocity_z,  # Up/down (usually 0 for altitude hold)
                        self.current_yaw_rate    # Yaw rotation
                    )
                else:
                    # Use attitude control for flex sensor input
                    self.publish_attitude()
            else:
                # Fallback to hover position control
                x_setpoint, y_setpoint, z_setpoint = self.get_hover_setpoint()
                self.publish_position_setpoint(x_setpoint, y_setpoint, z_setpoint)

        # Initial setup phase
        if self.setpoint_counter < 100:
            self.setpoint_counter += 1
            if self.setpoint_counter == 50:
                self.get_logger().info(
                    "Sending initial setpoints before enabling offboard..."
                )
            return

        # Flight state machine
        self.update_flight_state()
        self.setpoint_counter += 1

    def update_flight_state(self):
        """Update flight state machine"""

        if self.current_state.connected:
            # Handle RTL failsafe
            if self.current_state.mode == "AUTO.RTL":
                self.get_logger().error("🚨 Vehicle in RTL mode due to failsafe!")
                return

            # Re-enable offboard mode if switched out
            elif not self.offboard_enabled:
                if self.current_state.mode not in ["AUTO.RTL", "AUTO.LAND"]:
                    self.get_logger().warn(
                        f"Switching back to OFFBOARD from {self.current_state.mode}"
                    )
                    self.set_offboard_mode()

            # Re-arm if disarmed unexpectedly
            if self.offboard_enabled and not self.armed:
                self.get_logger().warn("Vehicle disarmed, attempting to re-arm")
                self.arm_vehicle()

        # State transitions
        if self.flight_state == "INIT":
            if self.offboard_enabled and self.armed:
                self.flight_state = "TAKEOFF"
                self.get_logger().info("🚁 Starting TAKEOFF sequence")

        elif self.flight_state == "TAKEOFF":
            if self.is_at_altitude(self.altitude):
                self.flight_state = "HOVER"
                self.hover_timer = 0
                current_alt = self.current_pose.pose.position.z
                self.get_logger().info(
                    f"✅ TAKEOFF complete! Altitude: {current_alt:.2f}m"
                )
                self.get_logger().info(
                    f"⏸️  HOVERING for {self.hover_duration * self.dt:.1f} seconds"
                )

        elif self.flight_state == "HOVER":
            self.hover_timer += 1
            if self.hover_timer >= self.hover_duration:
                self.flight_state = "FLEX_CONTROL"
                self.get_logger().info("🎮 Starting FLEX SENSOR OFFBOARD CONTROL")

                # Reset attitude control variables
                self.current_pitch_angle = 0.0
                self.current_yaw_angle = 0.0
                
                # Reset PID controllers to avoid windup
                self.pitch_pid.reset()
                self.yaw_pid.reset()

        elif self.flight_state == "FLEX_CONTROL":
            # Log status occasionally
            # if self.enable_flex_control:
            #     self.backup_and_disable_rate_i_gains()
            if self.setpoint_counter % 250 == 0:  # Every 5 seconds
                current_alt = self.current_pose.pose.position.z
                if self.use_velocity_control:
                    self.get_logger().info(
                        f"🎮 Velocity Control: Pitch_in={self.flex_pitch_input:.1f}, "
                        f"Yaw_in={self.flex_yaw_input:.1f}, Vel_X={self.target_velocity_x:.2f}m/s, "
                        f"Vel_Y={self.target_velocity_y:.2f}m/s, YawRate={self.current_yaw_rate:.1f}°/s, "
                        f"Alt={current_alt:.2f}m"
                    )
                else:
                    self.get_logger().info(
                        f"🎮 Attitude Control: Pitch_in={self.flex_pitch_input:.1f}, "
                        f"Yaw_in={self.flex_yaw_input:.1f}, Pitch={self.current_pitch_angle:.1f}°, "
                        f"Yaw={self.current_yaw_angle:.1f}°, YawRate={self.current_yaw_rate:.1f}°/s, "
                        f"Alt={current_alt:.2f}m"
                    )

        # Log status for other states
        if (
            self.flight_state in ["INIT", "TAKEOFF", "HOVER"]
            and self.setpoint_counter % 100 == 0
        ):
            current_alt = self.current_pose.pose.position.z
            self.get_logger().info(
                f"State: {self.flight_state}, Mode: {self.current_state.mode}, "
                f"Armed: {self.armed}, Alt: {current_alt:.2f}m"
            )
    # def destroy_node(self):
    #     """Clean up when shutting down"""
    #     # Restore I gains before shutting down
    #     if self.i_gains_modified:
    #         self.get_logger().info("Restoring rate controller I gains before shutdown...")
    #         self.restore_rate_i_gains()
        
    #     super().destroy_node()


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