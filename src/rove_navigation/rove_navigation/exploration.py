import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
from nav_msgs.msg import Odometry


class NavigateToFrontier(Node):
    def __init__(self):
        super().__init__("navigate_to_frontier_node")
        self.subscription = self.create_subscription(
            PointCloud2, "frontier_points", self.frontier_callback, 10
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, "/odometry/local", self.odometry_callback, 10
        )
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.min_frontier_size = (
            20  # Minimal number of points to consider a frontier valid
        )
        self.cluster_epsilon = (
            0.5  # Maximal distance between points in the same cluster
        )
        self.current_position = Point(x=0.0, y=0.0, z=0.0)  # Initial position

    def odometry_callback(self, msg):
        # Update current position based on odometry data
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z

    def frontier_callback(self, msg):
        # Parse the PointCloud2 message
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        # Filter and cluster the points into frontiers
        frontiers = self.process_frontiers(points)
        if not frontiers:
            self.get_logger().info("No valid frontiers found.")
            return
        # Calculate distances and find the closest frontier
        closest_frontier = self.find_closest_frontier(frontiers)
        if closest_frontier is not None:
            self.navigate_to_frontier(closest_frontier)

    def process_frontiers(self, points):
        # Convert points to numpy array for clustering
        points_array = np.array([[point[0], point[1]] for point in points])
        # DBSCAN clustering
        clustering = DBSCAN(
            eps=self.cluster_epsilon, min_samples=self.min_frontier_size
        ).fit(points_array)
        labels = clustering.labels_
        # Number of clusters, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        self.get_logger().info(f"Number of clusters found: {n_clusters_}")
        # Filter clusters to form frontiers
        frontiers = []
        for k in range(n_clusters_):
            class_member_mask = labels == k
            xy = points_array[class_member_mask]
            if len(xy) > self.min_frontier_size:
                frontiers.append(xy)
        return frontiers

    def find_closest_frontier(self, frontiers):
        min_distance = float("inf")
        closest_frontier = None
        for frontier in frontiers:
            centroid = np.mean(frontier, axis=0)
            distance = np.linalg.norm(
                centroid - np.array([self.current_position.x, self.current_position.y])
            )
            if distance < min_distance:
                min_distance = distance
                closest_frontier = frontier
        return closest_frontier

    def navigate_to_frontier(self, frontier):
        # Prepare the goal message and send it
        goal_pose = PoseStamped()
        goal_pose.header = Header(frame_id="map")

        x = float(np.mean(frontier[:, 0]))
        y = float(np.mean(frontier[:, 1]))

        goal_pose.pose.position = Point(x=x, y=y, z=0.0)
        goal_pose.pose.orientation.w = 1.0  # Assuming no orientation preference

        self.get_logger().info(f"Navigating to frontier at ({x}, {y})")
        self.action_client.wait_for_server()
        self.send_goal(goal_pose)

    def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info("Goal sent to navigation system.")

    def get_current_position(self):
        # Return the current position from the odometry data
        return self.current_position


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToFrontier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
