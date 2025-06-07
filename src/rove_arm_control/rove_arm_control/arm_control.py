import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from scipy.spatial.transform import Rotation as R

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

        twist_msg.linear.x = msg.axes[0]  # Y axis on the mouse (forward/backward motion)
        twist_msg.linear.y = -msg.axes[1]  # X axis on the mouse (left/right motion)
        twist_msg.linear.z = msg.axes[2]  # Z axis on the mouse (up/down motion)

        self.pose.pose.position.x += twist_msg.linear.x * self.SENSITIVITY
        self.pose.pose.position.y += twist_msg.linear.y * self.SENSITIVITY
        self.pose.pose.position.z += twist_msg.linear.z  * self.SENSITIVITY



        twist_msg.angular.x = msg.axes[3]  # Y rotation on the mouse (roll)
        twist_msg.angular.y = msg.axes[4]  # X rotation on the mouse (pitch)
        twist_msg.angular.z = -msg.axes[5]  # Z rotation on the mouse (yaw/twist)
        
        
        # Convert current orientation to scipy Rotation
        current_q = R.from_quat([
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w
        ])

        # Create a small delta rotation from input Euler angles
        # Note: Order depends on the convention, here it's 'xyz' = roll, pitch, yaw
        delta_euler = [
            twist_msg.angular.x * self.SENSITIVITY,
            twist_msg.angular.y * self.SENSITIVITY,
            twist_msg.angular.z * self.SENSITIVITY
        ]
        delta_q = R.from_euler('xyz', delta_euler)
        
        # Apply delta rotation
        new_q = current_q * delta_q  # Applies delta in local frame; reverse order for global frame

        # Convert back to quaternion
        x, y, z, w = new_q.as_quat()

        # Assign to pose
        self.pose.pose.orientation.x = x
        self.pose.pose.orientation.y = y
        self.pose.pose.orientation.z = z
        self.pose.pose.orientation.w = w
    
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
