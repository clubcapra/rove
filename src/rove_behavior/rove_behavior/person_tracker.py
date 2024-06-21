import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class Tracker(Node):
    MIN_ANG_VEL = 0.15
    MAX_ANG_VEL = 0.5
    ANGULAR_GAIN = 1.7

    def __init__(self):
        super().__init__('tracker')
        self.img_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.visualization_pub = self.create_publisher(
            Image,
            '/visualization',
            10)

        self.bridge = CvBridge()
        self.tracker = cv2.TrackerKCF_create()
        self.is_tracker_initialized = False

        self.get_logger().info("Node started!")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv_image.copy()
        obj = (0, 0, 0, 0)
        vel_msg = Twist()

        if not self.is_tracker_initialized:
            self.init_tracker(frame, obj)
        else:
            ok, obj = self.tracker.update(frame)
            if ok:
                self.designate_control(vel_msg, obj, msg.width)
                self.get_logger().info(f"Angular velocity: {vel_msg.angular.z:.2f}")
            else:
                self.get_logger().warn("Tracking failure detected. Stop vehicle!")
                cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

            self.vel_pub.publish(vel_msg)

            cv2.rectangle(frame, (int(obj[0]), int(obj[1])), (int(obj[0] + obj[2]), int(obj[1] + obj[3])), (255, 0, 0), 2, 1)

        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.visualization_pub.publish(img_msg)

    def init_tracker(self, frame, obj):
        obj = cv2.selectROI("ROI selector", frame, False)
        self.tracker.init(frame, obj)
        self.is_tracker_initialized = True
        cv2.destroyWindow("ROI selector")

    def designate_control(self, vel_msg, obj, img_width):
        obj_x_center = obj[0] + obj[2] / 2
        px_to_center = img_width / 2 - obj_x_center
        ang_vel = self.ANGULAR_GAIN * px_to_center / float(img_width)

        if ((ang_vel >= -self.MAX_ANG_VEL and ang_vel <= -self.MIN_ANG_VEL) or
            (ang_vel >= self.MIN_ANG_VEL and ang_vel <= self.MAX_ANG_VEL)):
            vel_msg.angular.z = ang_vel

def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
