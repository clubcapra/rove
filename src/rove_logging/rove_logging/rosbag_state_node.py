#!/usr/bin/env python3

import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class RosbagStatusPublisher(Node):
    def __init__(self):
        super().__init__('rosbag_status_publisher')
        self.publisher_ = self.create_publisher(Bool, 'is_rosbag_running', 10)
        timer_period = 1.0  # publish every 1 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Publishing is_rosbag_running = True')

        signal.signal(signal.SIGINT, self.shutdown_handler)
        signal.signal(signal.SIGTERM, self.shutdown_handler)

    def timer_callback(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        
    def shutdown_handler(self, signum, frame):
        # Publish False before shutting down
        self.get_logger().info('Shutting down: publishing is_rosbag_running = False')
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)

        # Give time for the message to be sent
        rclpy.spin_once(self, timeout_sec=0.5)

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RosbagStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown_handler(None, None)

if __name__ == '__main__':
    main()
