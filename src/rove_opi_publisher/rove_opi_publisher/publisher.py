#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path

class MinimalPublisher(Node):
  
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.publish_png)
        self.bridge = CvBridge()

    def publish_png(self):
        try:
            # Read PNG image
            img = cv2.imread("src/rove_opi_publisher/rove_opi_publisher/2.png", cv2.IMREAD_COLOR)
            # img = cv2.imread(str(Path("/src/rove_opi_publisher/2.png")))
            print(img)
            if img is None:
                self.get_logger().error('Failed to read the image file')
                return
            # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # Convert OpenCV image to ROS image message
            ros_img = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
            # Publish ROS image           
            self.publisher_.publish(ros_img)
        except Exception as e:
            self.get_logger().error('Error publishing PNG image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
