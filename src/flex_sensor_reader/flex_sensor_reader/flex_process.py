#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collections import deque
import numpy as np
from scipy import signal
from std_msgs.msg import Header, Float32MultiArray

from flex_sensor_reader.msg import FlexSensorData


class FlexSensorProcessor(Node):
    def __init__(self):
        super().__init__("flex_sensor_processor")

        # Declare parameters
        self.declare_parameter("input_topic", "flex_sensors/fake_raw")
        self.declare_parameter("output_topic", "flex_sensors/processed")
        self.declare_parameter("buffer_size", 30)
        self.declare_parameter("sampling_rate", 18.0)  # Match your publisher rate
        self.declare_parameter("butterworth_deg", 1)
        self.declare_parameter("cutoff_frequency", 0.8)
        self.declare_parameter("enable_calibration", True)
        self.declare_parameter("calibration_samples", 10)
        self.declare_parameter("enable_filter", True)
        # Get parameters
        self.input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.buffer_size = (
            self.get_parameter("buffer_size").get_parameter_value().integer_value
        )
        self.sampling_rate = (
            self.get_parameter("sampling_rate").get_parameter_value().double_value
        )
        self.butterworth_deg = (
            self.get_parameter("butterworth_deg").get_parameter_value().integer_value
        )
        self.cutoff_freq = (
            self.get_parameter("cutoff_frequency").get_parameter_value().double_value
        )
        self.enable_calibration = (
            self.get_parameter("enable_calibration").get_parameter_value().bool_value
        )
        self.calibration_samples = (
            self.get_parameter("calibration_samples")
            .get_parameter_value()
            .integer_value
        )
        self.enable_filter = (
            self.get_parameter("enable_filter").get_parameter_value().bool_value
        )

        # Initialize data structures
        self.num_sensors = 4
        self.data_buffers = [
            deque(maxlen=self.buffer_size) for _ in range(self.num_sensors)
        ]
        self.latest_values = [0.0 for _ in range(self.num_sensors)]
        self.raw_data_history = [deque(maxlen=1000) for _ in range(self.num_sensors)]

        # Calibration variables
        self.is_calibrating = self.enable_calibration
        self.calibration_data = [[] for _ in range(self.num_sensors)]
        self.initial_baseline_values = [0.0] * self.num_sensors
        self.baseline_values = [0.0] * self.num_sensors
        self.max_values = [1.0] * self.num_sensors
        self.min_values = [0.0] * self.num_sensors
        self.calibration_count = 0

        self.weight = [0.0 for _ in range(self.num_sensors)]

        # Filter design (Butterworth low-pass filter)
        if self.enable_filter:
            nyquist = self.sampling_rate / 2
            normalized_cutoff = self.cutoff_freq / nyquist
            self.filter_b, self.filter_a = signal.butter(
                self.butterworth_deg, normalized_cutoff, btype="low"
            )

            # Initialize filter states for each sensor
            self.filter_zi = [
                signal.lfilter_zi(self.filter_b, self.filter_a)
                for _ in range(self.num_sensors)
            ]

        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Float32MultiArray, self.input_topic, self.sensor_callback, 10
        )

        self.publisher = self.create_publisher(FlexSensorData, self.output_topic, 10)

        # Progressive processing stage publishers

        self.baseline_removed_publisher = self.create_publisher(
            FlexSensorData, "flex_sensors/baseline", 10
        )
        self.filter_publisher = self.create_publisher(
            FlexSensorData, "flex_sensors/filter", 10
        )

        self.get_logger().info("Flex sensor processor initialized")
        self.get_logger().info(f"Subscribing to: {self.input_topic}")
        self.get_logger().info(f"Publishing to: {self.output_topic}")
        if self.enable_filter:
            self.get_logger().info(
                f"Cutoff freq: {self.cutoff_freq}, Normalised: {normalized_cutoff}"
            )
            self.get_logger().info(f"Butterworth Degree: {self.butterworth_deg}")

        if self.is_calibrating:
            self.get_logger().info(
                f"Calibration mode enabled. Collecting {self.calibration_samples} samples..."
            )

    def sensor_callback(self, msg):
        """Main callback for processing incoming sensor data"""
        if len(msg.data) != self.num_sensors:
            self.get_logger().warn(
                f"Expected {self.num_sensors} sensors, got {len(msg.data)}"
            )
            return

        raw_data = list(msg.data)
        # Handle calibration phase
        if self.is_calibrating:
            self.collect_calibration_data(raw_data)
            return

        # Process the data
        processed_data = self.process_sensor_data(raw_data)

        # Create and publish final processed message
        self.publish_data(processed_data, self.publisher)

    def collect_calibration_data(self, raw_data):
        """Collect calibration data during initialization"""
        for i, value in enumerate(raw_data):
            self.calibration_data[i].append(value)

        self.calibration_count += 1

        if self.calibration_count >= self.calibration_samples:
            self.finalize_calibration()

    def finalize_calibration(self):
        """Finalize calibration by computing baseline and range values"""
        self.get_logger().info("Finalizing calibration...")

        for i in range(self.num_sensors):
            data = np.array(self.calibration_data[i])
            # Filter out infinite values
            finite_data = data[np.isfinite(data)]

            if len(finite_data) > 0:
                self.baseline_values[i] = np.mean(finite_data)

                self.min_values[i] = np.min(finite_data) - np.std(finite_data)
                self.max_values[i] = np.max(finite_data) + np.std(finite_data)
            else:
                self.baseline_values[i] = (
                    20000.0  # Typical unbent flex sensor resistance
                )
                self.min_values[i] = 10000.0  # Minimum flex sensor resistance
                self.max_values[i] = 160000.0  # Maximum flex sensor resistance

            self.get_logger().info(
                f"Sensor {i}: baseline={self.baseline_values[i]:.0f}Ω, "
                f"range=[{self.min_values[i]:.0f}Ω, {self.max_values[i]:.0f}Ω]"
            )
        self.initial_baseline_values = self.baseline_values.copy()
        self.is_calibrating = False
        self.get_logger().info("Calibration complete. Starting normal processing...")

    def process_sensor_data(self, raw_data):
        """Apply preprocessing to sensor data with progressive publishing"""
        processed_data = raw_data.copy()

        # Handle infinite resistance values (from your publisher)
        for i in range(len(processed_data)):
            if not np.isfinite(processed_data[i]):
                processed_data[i] = (
                    self.max_values[i] if hasattr(self, "max_values") else 160000.0
                )

        # Stage 1: Remove baseline (if calibrated)
        if self.enable_calibration and not self.is_calibrating:
            # Apply baseline removal with original baseline
            for i in range(self.num_sensors):
                processed_data[i] -= self.initial_baseline_values[i]

        self.publish_data(processed_data, self.baseline_removed_publisher)

        # Apply digital filter
        if self.enable_filter:
            processed_data = self.apply_filter(processed_data)

        self.publish_data(processed_data, self.filter_publisher)

        # Apply compensation
        processed_data = self.apply_compensation(processed_data)

        for i, value in enumerate(processed_data):
            self.data_buffers[i].append(value)

        return processed_data

    def apply_filter(self, data):
        """Apply Butterworth low-pass filter"""
        filtered_data = []

        for i, value in enumerate(data):
            # Apply filter using lfilter with initial conditions
            filtered_value, self.filter_zi[i] = signal.lfilter(
                self.filter_b, self.filter_a, [value], zi=self.filter_zi[i]
            )
            filtered_data.append(filtered_value[0])

        return filtered_data

    def apply_compensation(self, data):
        T_low, T_high = 80, 230
        beta = 0.35
        compensate_data = []

        for i, value in enumerate(data):
            prev_data = self.data_buffers[i][-1] if self.data_buffers[i] else 0.0
            dx = (value - prev_data) * self.sampling_rate
            print(dx, i)

            if (abs(value)) <= 0.03 * self.initial_baseline_values[i]:
                if self.weight[i] == 1:  # after contact
                    raw = (abs(dx) - T_low) / max(T_high - T_low, 1e-6)
                    raw = np.clip(raw, 0, 1)
                    self.weight[i] = (1 - beta) * self.weight[i] + beta * raw
                    new_baseline_values = self.weight[i] * self.baseline_values[i] + (
                        1 - self.weight[i]
                    ) * (value + self.baseline_values[i])

                    value += self.baseline_values[i] - new_baseline_values
                    self.baseline_values[i] = new_baseline_values
                else:
                    self.weight[i] = 1
            compensate_data.append(value)
        print()
        return compensate_data

    def publish_data(self, data, publisher):
        """Publish data from a specific processing stage"""

        msg = FlexSensorData()
        msg.resistance_values = [float(x) for x in data]  # Ensure all values are float

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "flex_sensors"

        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        processor = FlexSensorProcessor()

        # Use MultiThreadedExecutor for better performance
        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor()
        executor.add_node(processor)

        processor.get_logger().info("Flex sensor processor running...")
        executor.spin()

    except KeyboardInterrupt:
        processor.get_logger().info("Shutting down flex sensor processor...")
    finally:
        if "processor" in locals():
            processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
