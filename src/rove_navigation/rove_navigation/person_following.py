import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PointStamped
import math

class NavigateToPersonNode(Node):
    def __init__(self, truncate_distance):
        super().__init__('navigate_to_person')
        self.subscription = self.create_subscription(
            PointStamped,
            '/person_position',
            self.navigate_to_person,
            10)
        
        self.goal_update_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Distance to truncate from the target position
        self.truncate_distance = truncate_distance

    def navigate_to_person(self, msg):
        # Ensure that the transformation is available
        now = rclpy.time.Time()
        if self.tf_buffer.can_transform('map', msg.header.frame_id, now):
            point_transformed = self.tf_buffer.transform(msg, 'map', timeout=rclpy.duration.Duration(seconds=1))
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()

            # Calculate the angle for the truncated goal using atan2
            angle = math.atan2(point_transformed.point.y, point_transformed.point.x)
            
            # Calculate the position truncating the specified distance from the transformed position
            goal_pose.pose.position.x = point_transformed.point.x - self.truncate_distance * math.cos(angle)
            goal_pose.pose.position.y = point_transformed.point.y - self.truncate_distance * math.sin(angle)
            goal_pose.pose.position.z = 0.0  # Normally zero for ground robots

            goal_pose.pose.orientation.w = 1.0  # No rotation about the z-axis

            # Log the navigation target for debugging
            self.get_logger().info(f'Navigating to truncated goal: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}')
            
            # Publish the goal
            self.goal_update_pub.publish(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    # You can adjust the truncate distance here
    node = NavigateToPersonNode(truncate_distance=1.5)  # for example, truncate 1.5 meters
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
