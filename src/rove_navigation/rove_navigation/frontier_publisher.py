import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
from queue import Queue
from threading import Lock
import sensor_msgs_py.point_cloud2 as pc2


class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.frontier_pub = self.create_publisher(
            PointCloud2, 'frontier_points', 10)

        self.map_data = None
        self.map_lock = Lock()

    def map_callback(self, msg):
        with self.map_lock:
            self.map_data = msg
        self.process_map_data()

    def process_map_data(self):
        if self.map_data is None:
            return

        frontiers = self.find_frontiers()
        if frontiers:
            largest_frontier = max(frontiers, key=len)
            self.publish_frontier(largest_frontier)
        else:
            self.get_logger().info('No frontiers detected.')

    def find_frontiers(self):
        with self.map_lock:
            map_array = np.array(self.map_data.data).reshape(
                (self.map_data.info.height, self.map_data.info.width))

        frontiers = []
        visited = np.zeros_like(map_array, dtype=bool)
        frontier_flag = np.zeros_like(map_array, dtype=bool)

        for y in range(map_array.shape[0]):
            for x in range(map_array.shape[1]):
                if map_array[y, x] == 0 and not visited[y, x]:  # Free space
                    queue = Queue()
                    queue.put((x, y))
                    new_frontier = []
                    while not queue.empty():
                        px, py = queue.get()
                        if visited[py, px]:
                            continue
                        visited[py, px] = True
                        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-connectivity
                            nx, ny = px + dx, py + dy
                            if 0 <= nx < map_array.shape[1] and 0 <= ny < map_array.shape[0]:
                                if map_array[ny, nx] == -1 and not frontier_flag[ny, nx]:  # Unknown space
                                    frontier_flag[ny, nx] = True
                                    new_frontier.append((nx, ny))
                                elif map_array[ny, nx] == 0 and not visited[ny, nx]:
                                    queue.put((nx, ny))
                    if new_frontier:
                        frontiers.append(new_frontier)
        return frontiers

    def publish_frontier(self, frontier):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_data.header.frame_id
        points = [(x * self.map_data.info.resolution + self.map_data.info.origin.position.x,
                   y * self.map_data.info.resolution + self.map_data.info.origin.position.y, 0.0) for x, y in frontier]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]        
        cloud = pc2.create_cloud(header,fields, points)
        self.frontier_pub.publish(cloud)
        self.get_logger().info(f'Published {len(frontier)} points representing the largest frontier.')

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
