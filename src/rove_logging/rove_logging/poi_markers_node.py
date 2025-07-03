import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


class PoiMarkersNode(Node):
    def __init__(self):
        super().__init__('poi_markers_node')

        self.points = []  # List of (x, y, z) tuples

        # Subscribe to RViz clicked points
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)

        # Publisher for PointCloud2
        self.publisher = self.create_publisher(PointCloud2, '/poi_cloud', 10)

        # Timer to republish the cloud every second
        self.create_timer(1.0, self.publish_cloud)

    def clicked_point_callback(self, msg: PointStamped):
        point = (msg.point.x, msg.point.y, msg.point.z)
        self.points.append(point)
        self.get_logger().info(f'Point added: {point} (total: {len(self.points)})')

    def publish_cloud(self):
        if not self.points:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Make sure RViz uses the same frame

        cloud = point_cloud2.create_cloud_xyz32(header, self.points)
        self.publisher.publish(cloud)
        self.get_logger().info(f'Published PointCloud2 with {len(self.points)} points')
        # self.export_2d_map()

    def export_2d_map(self):
        return

def main():
    rclpy.init()
    node = PoiMarkersNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
