#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from flex_sensor_reader.msg import FlexSensorData  # update with your package


class FakeFlexSensor(Node):
    def __init__(self):
        super().__init__("fake_flex_sensor")

        self.publisher_ = self.create_publisher(
            FlexSensorData, "/flex_sensors/processed", 10
        )

        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.t = 0.0
        self.amplitude = 250.0
        self.freq = 0.2

        self.get_logger().info("Fake Flex Sensor publisher started")

    def timer_callback(self):
        msg = FlexSensorData()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        value = 1000.0 + self.amplitude * math.sin(2.0 * math.pi * self.freq * self.t)
        msg.resistance_values = [value] * 4  # same sine wave on all channels
        msg.resistance_values[1] = 3000.0
        msg.resistance_values[2] = 3000.0
        self.publisher_.publish(msg)
        self.t += self.timer_period


def main(args=None):
    rclpy.init(args=args)
    node = FakeFlexSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
