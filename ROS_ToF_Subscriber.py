#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DroneListener(Node):
    def __init__(self):
        super().__init__('tof_listener')

        # Subscriber to Float32 distance data
        self.create_subscription(Float32, 'track_distance', self.dist_cb, 10)

    def dist_cb(self, msg):
        self.get_logger().info(f"[Distance] = {msg.data:.6f} m")


def main(args=None):
    rclpy.init(args=args)
    node = DroneListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
