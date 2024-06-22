import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PointStamped  # This is necessary for the transform
import math

class NavigateToPersonNode(Node):
    def __init__(self):
        super().__init__('navigate_to_person')
        self.subscription = self.create_subscription(
            PointStamped,
            '/person_position',
            self.navigate_to_person,
            10)

        self.goal_update_pub = self.create_publisher(
            PoseStamped,
            '/goal_update',
            10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def navigate_to_person(self, msg):

        # Ensure that the transformation is available
        now = rclpy.time.Time()
        if self.tf_buffer.can_transform('map', msg.header.frame_id, now):
            # Properly use tf2_geometry_msgs do_transform function
            point_transformed = self.tf_buffer.transform(msg, 'map', timeout=rclpy.duration.Duration(seconds=1))
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()

            # Set the transformed position as the goal
            goal_pose.pose.position.x = point_transformed.point.x
            goal_pose.pose.position.y = point_transformed.point.y
            goal_pose.pose.position.z = 0.0  # Normally zero for ground robots

            goal_pose.pose.orientation.w = 1.0  # No rotation about the z-axis

            # Log the navigation target for debugging
            self.get_logger().info(f'Navigating to transformed goal: {goal_pose.pose.position.x} {goal_pose.pose.position.y}...')
            
            # Publish the goal
            self.goal_update_pub.publish(goal_pose)
            

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPersonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
