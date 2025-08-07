#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from collections import deque
import numpy as np
from scipy import signal


class FlexSensorProcessor(Node):
    def __init__(self):
        super().__init__("flex_sensor_processor")

        # Declare parameters
        self.declare_parameter("input_topic", "flex_sensors/raw")
        self.declare_parameter("output_topic", "flex_sensors/processed")
        self.declare_parameter("buffer_size", 10)
        self.declare_parameter("sampling_rate", 50.0)  # Match your publisher rate
        self.declare_parameter("cutoff_frequency", 10.0)  # Lower for 50Hz sampling
        self.declare_parameter("enable_calibration", True)
        self.declare_parameter("calibration_samples", 500)
        self.declare_parameter("enable_filter", True)
        self.declare_parameter("enable_drift_compensation", True)  # Handle TPU creep
        self.declare_parameter(
            "drift_update_rate", 0.005
        )  # Faster adaptation (0.5% per sample)
        self.declare_parameter(
            "drift_threshold", 50.0
        )  # More sensitive to quiet periods
        self.declare_parameter(
            "drift_window_size", 50
        )  # Smaller window for faster response
        self.declare_parameter("enable_median_filter", True)
        self.declare_parameter("median_window", 3)
        self.declare_parameter("median_filter_size", 10)  # Median filter size

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
        self.enable_drift_compensation = (
            self.get_parameter("enable_drift_compensation")
            .get_parameter_value()
            .bool_value
        )
        self.drift_update_rate = (
            self.get_parameter("drift_update_rate").get_parameter_value().double_value
        )
        self.drift_threshold = (
            self.get_parameter("drift_threshold").get_parameter_value().double_value
        )
        self.drift_window_size = (
            self.get_parameter("drift_window_size").get_parameter_value().integer_value
        )
        self.enable_median_filter = (
            self.get_parameter("enable_median_filter").get_parameter_value().bool_value
        )
        self.median_window = (
            self.get_parameter("median_window").get_parameter_value().integer_value
        )
        self.median_filter_size = (
            self.get_parameter("median_filter_size").get_parameter_value().integer_value
        )
        # Initialize data structures
        self.num_sensors = 4
        self.data_buffers = [
            deque(maxlen=self.buffer_size) for _ in range(self.num_sensors)
        ]
        self.raw_data_history = [deque(maxlen=1000) for _ in range(self.num_sensors)]

        # Drift compensation buffers
        if self.enable_drift_compensation:
            self.drift_windows = [
                deque(maxlen=self.drift_window_size) for _ in range(self.num_sensors)
            ]
            self.last_drift_update = [0] * self.num_sensors
            self.drift_detection_counter = 0
            self.original_baseline_values = [
                0.0
            ] * self.num_sensors  # Store original baselines

        # Calibration variables
        self.is_calibrating = self.enable_calibration
        self.calibration_data = [[] for _ in range(self.num_sensors)]
        self.baseline_values = [0.0] * self.num_sensors
        self.max_values = [1.0] * self.num_sensors
        self.min_values = [0.0] * self.num_sensors
        self.calibration_count = 0
        self.current_normal = 0.0
        self.current_balance = 0.0

        # Filter design (Butterworth low-pass filter)
        if self.enable_filter:
            nyquist = self.sampling_rate / 2
            normalized_cutoff = self.cutoff_freq / nyquist
            self.filter_b, self.filter_a = signal.butter(
                1, normalized_cutoff, btype="low"
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

        self.publisher = self.create_publisher(Float32MultiArray, self.output_topic, 10)

        # Progressive processing stage publishers

        self.baseline_removed_publisher = self.create_publisher(
            Float32MultiArray, "flex_sensors/stage_1_baseline_removed", 10
        )

        self.drift_compensation_publisher = self.create_publisher(
            Float32MultiArray, "flex_sensors/stage_1b_drift_compensation", 10
        )

        self.median_publisher = self.create_publisher(
            Float32MultiArray, "flex_sensors/stage_1c_median_filter", 10
        )

        self.filtered_publisher = self.create_publisher(
            Float32MultiArray, "flex_sensors/stage_2_filtered", 10
        )

        self.smoothed_publisher = self.create_publisher(
            Float32MultiArray, "flex_sensors/stage_3_smoothed", 10
        )

        # Publishers for flex sensor signals
        # self.normal_publisher = self.create_publisher(
        #     Float32, "flex_sensors/normal", 10
        # )

        # self.balance_publisher = self.create_publisher(
        #     Float32, "flex_sensors/balance", 10
        # )

        # Statistics publisher (optional)
        self.stats_publisher = self.create_publisher(
            Float32MultiArray, "flex_sensors/statistics", 10
        )

        # Create timer for statistics publishing
        self.stats_timer = self.create_timer(1.0, self.publish_statistics)

        self.get_logger().info("Flex sensor processor initialized")
        self.get_logger().info(f"Subscribing to: {self.input_topic}")
        self.get_logger().info(f"Publishing to: {self.output_topic}")
        
        if self.enable_drift_compensation:
            self.get_logger().info(
                f"Drift compensation enabled: update_rate={self.drift_update_rate:.4f}, threshold={self.drift_threshold:.0f}Ω, window={self.drift_window_size} samples"
            )
        if self.enable_median_filter:
            self.get_logger().info(
                f"Median filter enabled: window_size={self.median_window:.3f}"
            )
        self.get_logger().info("Progressive stage topics:")
        self.get_logger().info("  - flex_sensors/stage_0_raw")
        self.get_logger().info("  - flex_sensors/stage_1_baseline_removed")
        if self.enable_drift_compensation:
            self.get_logger().info("  - flex_sensors/stage_1b_drift_compensation")
        if self.enable_median_filter:
            self.get_logger().info("  - flex_sensors/stage_1c_median_filter")
        self.get_logger().info("  - flex_sensors/stage_2_filtered")
        self.get_logger().info("  - flex_sensors/stage_3_smoothed")

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

        # Store raw data for statistics and drift compensation
        for i, value in enumerate(raw_data):
            self.raw_data_history[i].append(value)
            if self.enable_drift_compensation and hasattr(self, "drift_windows"):
                self.drift_windows[i].append(value)

        # Handle calibration phase
        if self.is_calibrating:
            self.collect_calibration_data(raw_data)
            return

        # Process the data
        processed_data = self.process_sensor_data(raw_data)

        # Create and publish final processed message
        processed_msg = Float32MultiArray()
        processed_msg.data = processed_data
        self.publisher.publish(processed_msg)

        # # Extract sensor data
        # sensor_0 = processed_msg.data[0]
        # sensor_3 = processed_msg.data[3]

        # # Calculate normal and balance signals
        # self.current_normal = sensor_0 + sensor_3
        # self.current_balance = sensor_0 - sensor_3

        # # Publish normal and balance signals
        # normal_msg = Float32()
        # normal_msg.data = self.current_normal
        # self.normal_publisher.publish(normal_msg)

        # balance_msg = Float32()
        # balance_msg.data = self.current_balance
        # self.balance_publisher.publish(balance_msg)

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
                # Fallback values for flex sensor resistance (unbent ~25kΩ, bent ~75kΩ)
                self.baseline_values[i] = (
                    25000.0  # Typical unbent flex sensor resistance
                )
                self.min_values[i] = 10000.0  # Minimum flex sensor resistance
                self.max_values[i] = 160000.0  # Maximum flex sensor resistance

            # Store original baseline for drift compensation
            if self.enable_drift_compensation:
                self.original_baseline_values[i] = self.baseline_values[i]

            self.get_logger().info(
                f"Sensor {i}: baseline={self.baseline_values[i]:.0f}Ω, "
                f"range=[{self.min_values[i]:.0f}Ω, {self.max_values[i]:.0f}Ω]"
            )

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

        # Stage 1: Remove baseline (if calibrated) and handle drift
        if self.enable_calibration and not self.is_calibrating:
            # Apply baseline removal with original baseline
            for i in range(self.num_sensors):
                processed_data[i] -= self.baseline_values[i]

            # Update baseline for drift compensation AFTER showing baseline removal
            if self.enable_drift_compensation:
                self.update_baseline_for_drift(
                    raw_data
                )  # Use raw data for drift analysis

        self.publish_stage_data(
            processed_data, self.baseline_removed_publisher, "Baseline Removed"
        )

        # Stage 1b: Show drift compensation signal
        if (
            self.enable_drift_compensation
            and self.enable_calibration
            and not self.is_calibrating
        ):
            drift_compensation_data = self.get_drift_compensation_signal()
            self.publish_stage_data(
                drift_compensation_data,
                self.drift_compensation_publisher,
                "Drift Compensation",
            )

        # Stage 1c: Apply median filter to remove spikes
        if self.enable_median_filter:
            processed_data = self.apply_median_filter(processed_data)
            self.publish_stage_data(
                processed_data, self.median_publisher, "Median Filtered"
            )

        # Stage 2: Apply digital filter
        if self.enable_filter:
            processed_data = self.apply_filter(processed_data)
        self.publish_stage_data(processed_data, self.filtered_publisher, "Filtered")

        # Stage 3: Store in buffers for additional processing
        for i, value in enumerate(processed_data):
            self.data_buffers[i].append(value)

        # Stage 4: Apply additional smoothing
        processed_data = self.apply_smoothing(processed_data)
        self.publish_stage_data(processed_data, self.smoothed_publisher, "Smoothed")

        return processed_data

    def apply_smoothing(self, data):
        """Apply moving average smoothing"""
        smoothed_data = []

        for i, current_value in enumerate(data):
            if len(self.data_buffers[i]) >= 3:  # Need at least 3 samples
                # Simple moving average
                buffer_array = np.array(list(self.data_buffers[i]))
                smoothed_value = np.mean(buffer_array[-3:])  # Last 3 samples
                smoothed_data.append(smoothed_value)
            else:
                smoothed_data.append(current_value)

        return smoothed_data

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

    def publish_statistics(self):
        """Publish statistics about the sensor data"""
        if not self.raw_data_history[0] or len(self.raw_data_history[0]) < 10:
            return

        stats_data = []

        for i in range(self.num_sensors):
            if len(self.raw_data_history[i]) > 0:
                data_array = np.array(list(self.raw_data_history[i]))
                mean_val = np.mean(data_array)
                std_val = np.std(data_array)
                min_val = np.min(data_array)
                max_val = np.max(data_array)

                # Pack stats as [mean, std, min, max] for each sensor
                stats_data.extend([mean_val, std_val, min_val, max_val])

        if stats_data:
            stats_msg = Float32MultiArray()
            stats_msg.data = stats_data
            self.stats_publisher.publish(stats_msg)

    def get_sensor_statistics(self):
        """Get current sensor statistics"""
        stats = {}
        for i in range(self.num_sensors):
            if len(self.raw_data_history[i]) > 0:
                data = np.array(list(self.raw_data_history[i]))
                stats[f"sensor_{i}"] = {
                    "mean": np.mean(data),
                    "std": np.std(data),
                    "min": np.min(data),
                    "max": np.max(data),
                    "samples": len(data),
                }
        return stats

    def publish_stage_data(self, data, publisher, stage_name):
        """Publish data from a specific processing stage"""
        try:
            msg = Float32MultiArray()
            msg.data = [float(x) for x in data]  # Ensure all values are float
            publisher.publish(msg)

            # Occasional debug logging
            if hasattr(self, "_debug_counter"):
                self._debug_counter += 1
            else:
                self._debug_counter = 0

            if self._debug_counter % 250 == 0:  # Log every 5 seconds at 50Hz
                self.get_logger().debug(
                    f"{stage_name}: {[f'{x:.0f}' for x in data[:2]]}..."
                )  # Show first 2 sensors

        except Exception as e:
            self.get_logger().error(f"Error publishing {stage_name} data: {str(e)}")

    def update_baseline_for_drift(self, current_data):
        """Update baseline values to compensate for TPU creep drift"""
        self.drift_detection_counter += 1

        # Check for drift more frequently for faster response
        if self.drift_detection_counter % 10 != 0:  # Check every 0.2 seconds at 50Hz
            return

        for i in range(self.num_sensors):
            if len(self.drift_windows[i]) < self.drift_window_size:
                continue

            # Calculate recent average and rate of change
            recent_window = list(self.drift_windows[i])
            recent_avg = np.mean(recent_window)

            # Calculate rate of change (much better than std!)
            if len(recent_window) >= 10:  # Need at least 10 samples
                # Calculate rate of change over last 10 samples (0.2 seconds)
                recent_values = recent_window[-10:]
                rate_of_change = abs(recent_values[-1] - recent_values[0]) / len(
                    recent_values
                )  # Ohms per sample

                # Also check maximum rate of change in the window
                max_rate_of_change = max(
                    abs(recent_values[j] - recent_values[j - 1])
                    for j in range(1, len(recent_values))
                )
            else:
                rate_of_change = float("inf")  # Force skip if not enough data
                max_rate_of_change = float("inf")

            # Detect "quiet" period using rate of change instead of std
            is_quiet_period = (
                rate_of_change < self.drift_threshold
                and max_rate_of_change < self.drift_threshold * 2
            )

            # Also detect large systematic drift
            expected_resistance = self.baseline_values[i]
            drift_amount = recent_avg - expected_resistance
            large_drift = abs(drift_amount) > (
                self.drift_threshold * 4
            )  # 4x threshold for large drift

            if is_quiet_period or large_drift:
                # More aggressive update threshold for large drifts
                min_update_threshold = (
                    self.drift_threshold * 1.0
                    if is_quiet_period
                    else self.drift_threshold * 0.5
                )

                # Update if drift is significant
                if abs(drift_amount) > min_update_threshold:
                    # Use higher update rate for large drifts
                    update_rate = self.drift_update_rate
                    if large_drift:
                        update_rate *= 3  # 3x faster for large drifts

                    # Gradually update baseline (adaptive filtering)
                    baseline_update = drift_amount * update_rate
                    old_baseline = self.baseline_values[i]
                    self.baseline_values[i] += baseline_update

                    # Log significant baseline changes
                    if abs(baseline_update) > 3.0:  # Log changes > 3 Ohms
                        quiet_indicator = "QUIET" if is_quiet_period else "LARGE_DRIFT"
                        self.get_logger().info(
                            f"Sensor {i} baseline drift compensation ({quiet_indicator}): "
                            f"{old_baseline:.0f}Ω → {self.baseline_values[i]:.0f}Ω "
                            f"(drift: {drift_amount:.0f}Ω, update: {baseline_update:.1f}Ω, "
                            f"rate: {rate_of_change:.1f}Ω/sample, max_rate: {max_rate_of_change:.1f}Ω/sample)"
                        )

                    self.last_drift_update[i] = self.drift_detection_counter

    def get_drift_compensation_signal(self):
        """Get the current drift compensation values for each sensor"""
        drift_compensation = []

        for i in range(self.num_sensors):
            if hasattr(self, "original_baseline_values"):
                # Calculate how much the baseline has drifted from original
                drift_amount = (
                    self.baseline_values[i] - self.original_baseline_values[i]
                )
                drift_compensation.append(drift_amount)
            else:
                drift_compensation.append(0.0)

        return drift_compensation

    def apply_median_filter(self, data):
        """Apply median filter to remove outliers and spikes from sensor data."""

        # Initialize buffers if needed
        if not hasattr(self, "median_filter_buffers"):
            self.median_filter_buffers = [
                deque(maxlen=self.median_filter_size) for _ in range(self.num_sensors)
            ]
            self.get_logger().info("Initialized median filter buffers")

        median_filtered_data = []

        for i, value in enumerate(data):
            buffer = self.median_filter_buffers[i]
            buffer.append(value)

            if len(buffer) >= self.median_window:
                median = np.median(buffer)
            else:
                median = value  # Not enough samples yet

            median_filtered_data.append(median)

        return median_filtered_data


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
