import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import math

class NavigateToPersonNode(Node):
    def __init__(self):
        super().__init__('navigate_to_person')
        self.subscription = self.create_subscription(
            PointStamped,
            '/person_position',
            self.navigate_to_person,
            10)
        self.navigator = BasicNavigator()

    def navigate_to_person(self, msg):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        self.navigator.goToPose(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPersonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
