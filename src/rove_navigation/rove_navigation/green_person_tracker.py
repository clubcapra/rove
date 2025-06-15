#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class Tracker(Node):
    def __init__(self):
        super().__init__("tracker")
        self.img_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/zed/zed_node/depth/depth_registered", self.depth_callback, 10
        )
        self.visualization_pub = self.create_publisher(
            Image, "/green_person_bounding_box", 10
        )
        self.position_pub = self.create_publisher(PointStamped, "/person_position", 10)

        self.bridge = CvBridge()
        self.depth_image = None
        self.get_logger().info("Node started!")

        # Camera intrinsic parameters
        self.fx = 363  # Focal length in x axis
        self.fy = 363  # Focal length in y axis
        self.cx = 672  # Optical center x coordinate
        self.cy = 188  # Optical center y coordinate

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {str(e)}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv_image.copy()

            # Convert the frame to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the range for green color in HSV (expanded range)
            lower_green = np.array([30, 40, 40])
            upper_green = np.array([90, 255, 255])

            # Create a mask for green color
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour (assuming it is the object to track)
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)

                # Draw bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # Get the center of the bounding box
                center_x = x + w // 2
                center_y = y + h // 2

                # Publish the position if depth image is available
                if self.depth_image is not None:
                    depth = float(self.depth_image[center_y, center_x])
                    if np.isfinite(depth):
                        self.publish_position(center_x, center_y, depth, msg.header)
            else:
                self.get_logger().warn("No green object detected!")
                cv2.putText(
                    frame,
                    "No green object detected",
                    (100, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2,
                )

            # Convert the frame back to an Image message and publish it
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.visualization_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")

    def publish_position(self, x, y, depth, header):
        # Convert pixel coordinates to camera coordinates
        x_cam = (x - self.cx) * depth / self.fx
        y_cam = (y - self.cy) * depth / self.fy
        z_cam = depth

        point = PointStamped()
        point.header = header
        point.point.x = x_cam
        point.point.y = y_cam
        point.point.z = z_cam

        # Transform the point to the base_link frame
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", header.frame_id, rclpy.time.Time()
            )
            point_transformed = do_transform_point(point, transform)
            self.position_pub.publish(point_transformed)
            self.get_logger().info(
                f"Person (green square) position in base_link: x={point_transformed.point.x}, y={point_transformed.point.y}, z={point_transformed.point.z}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to transform point: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
