import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped


class RoveArmControl(Node):
    
    def __init__(self):
        super().__init__('joy_listener')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(PoseStamped, '/arm_target', 50)
        self.pose : PoseStamped = PoseStamped()
        self.lastPose: PoseStamped = self.pose
        self.lastPose.pose.position.x += 1
        self.pose.header.frame_id = "map"
        self.SENSITIVITY = 0.01
    
    def poses_are_equal(self, pose1: PoseStamped, pose2: PoseStamped):
        return (pose1.pose.orientation.x == pose2.pose.orientation.x and pose1.pose.orientation.y == pose2.pose.orientation.y and
                pose1.pose.orientation.z == pose2.pose.orientation.z and pose1.pose.position.x == pose2.pose.position.x and
                pose1.pose.position.y == pose2.pose.position.y and pose1.pose.position.z == pose2.pose.position.z )

    def joy_callback(self, msg):
        # Map the joystick axes to linear and angular velocities
        twist_msg = Twist()

        # Assuming axes[0] = X (forward/backward), axes[1] = Y (left/right), axes[3] = pitch (rotation)
        twist_msg.linear.x = msg.axes[0]  # Forward/backward motion (X-axis)
        twist_msg.linear.y = msg.axes[1]  # Left/right motion (Y-axis)
        twist_msg.linear.z = msg.axes[2]  # Up/down motion (Z-axis)
        twist_msg.angular.x = msg.axes[3]  # Rotation in pitch
        twist_msg.angular.y = msg.axes[4]  # Rotation in roll
        twist_msg.angular.z = msg.axes[5]  # Rotation in yaw (twist)
        
        self.pose.pose.position.x += twist_msg.linear.x * self.SENSITIVITY
        self.pose.pose.position.y += twist_msg.linear.y * self.SENSITIVITY
        self.pose.pose.position.z += twist_msg.linear.z  * self.SENSITIVITY
        self.pose.pose.orientation.x += twist_msg.angular.x * self.SENSITIVITY
        self.pose.pose.orientation.y += twist_msg.angular.y * self.SENSITIVITY
        self.pose.pose.orientation.z += twist_msg.angular.z * self.SENSITIVITY
    
        self.cmd_vel_publisher.publish(self.pose)
        # self.get_logger().info(f'Pose: pose.position={self.pose.pose.position}, pose.orientation={self.pose.pose.orientation}')
        


def main(args=None):
    rclpy.init(args=args)
    node = RoveArmControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down Rove Arm Control.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
