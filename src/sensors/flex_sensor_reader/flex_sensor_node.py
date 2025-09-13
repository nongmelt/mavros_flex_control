#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import ChannelFloat32
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time


class FlexSensorReader(Node):
    def __init__(self):
        super().__init__("flex_sensor_reader")

        # Initialize I2C and ADS1115
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.ads = ADS.ADS1115(self.i2c, data_rate=860)

            # Initialize analog input channels for 4 flex sensors
            self.channels = [
                AnalogIn(self.ads, ADS.P0),  # Channel 0
                AnalogIn(self.ads, ADS.P1),  # Channel 1
                AnalogIn(self.ads, ADS.P2),  # Channel 2
                AnalogIn(self.ads, ADS.P3),  # Channel 3
            ]

            self.get_logger().info("ADS1115 initialized successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize ADS1115: {str(e)}")
            return

        # Create publisher for flex sensor data using ChannelFloat32 (includes header with timestamp)
        self.publisher_ = self.create_publisher(
            ChannelFloat32, "flex_sensors/raw", 50
        )

        # Create timer for publishing sensor data
        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.read_and_publish_sensors)

        # Parameters
        self.declare_parameter("reference_voltage", 3.3)
        self.declare_parameter("divider_resistance", 47000.0)  # 47K ohm

        # Initialize sequence counter for header
        self.sequence_counter = 0

        self.get_logger().info("Flex sensor reader node initialized")

    def read_and_publish_sensors(self):
        try:
            # Read all 4 flex sensors
            resistance_values = []
            voltage_values = []

            for i, channel in enumerate(self.channels):
                voltage = channel.voltage
                voltage_values.append(voltage)

                # Calculate resistance using voltage divider formula
                # R_flex = R_fixed * (V_ref - V_measured) / V_measured
                ref_voltage = self.get_parameter("reference_voltage").value
                divider_resistance = self.get_parameter("divider_resistance").value

                if voltage > 0:
                    flex_resistance = (
                        divider_resistance * (ref_voltage - voltage) / voltage
                    )
                else:
                    flex_resistance = float("inf")

                resistance_values.append(flex_resistance)

                # Log occasional readings for debugging
                if i == 0:  # Only log first sensor to avoid spam
                    self.get_logger().debug(
                        f"Sensor {i}: {voltage:.3f}V, {flex_resistance:.0f}Ω"
                    )

            # Create message with header (includes timestamp)
            msg = ChannelFloat32()
            
            # Create and populate header with timestamp
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "flex_sensors"
            msg.header.seq = self.sequence_counter
            self.sequence_counter += 1

            # Set the sensor data
            msg.name = "flex_sensor_resistance"
            msg.values = resistance_values

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {str(e)}")

            # Publish error message with timestamp
            msg = ChannelFloat32()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "flex_sensors"
            msg.header.seq = self.sequence_counter
            self.sequence_counter += 1
            
            msg.name = "flex_sensor_resistance"
            msg.values = [0.0, 0.0, 0.0, 0.0]

            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    flex_sensor_reader = FlexSensorReader()

    try:
        rclpy.spin(flex_sensor_reader)
    except KeyboardInterrupt:
        pass
    finally:
        flex_sensor_reader.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()