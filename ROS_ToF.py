#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class ToFCommandNode(Node):
    def __init__(self):
        super().__init__('tof_command_node')

        # Publisher to publish distance as Float32
        self.dist_pub = self.create_publisher(Float32, 'track_distance', 10)

        # Timer to simulate periodic distance publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_distance)

    #change this to interface with the ToF data from the sensor, figure out how to get that done 
    def publish_distance(self):
        # Simulate or replace this with real sensor data
        distance = random.uniform(0.2, 2.0)  # Simulated distance in meters

        msg = Float32()
        msg.data = distance

        self.dist_pub.publish(msg)
        self.get_logger().info(f'Published distance: {distance:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = ToFCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
