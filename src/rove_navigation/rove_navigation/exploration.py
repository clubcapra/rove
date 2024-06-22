import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import numpy as np
from queue import Queue
from threading import Lock

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')

        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.map_data = None
        self.map_lock = Lock()

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.explore_timer = self.create_timer(1.0, self.explore_callback)

        self.map_received = False

    def map_callback(self, msg):
        with self.map_lock:
            self.map_data = msg
            self.map_received = True


    
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

        
    def explore_callback(self):
        if not self.map_received:
            self.get_logger().info('Waiting for map data...')
            return

        frontiers = self.find_frontiers()
        if frontiers:
            best_frontier = self.select_best_frontier(frontiers)
            self.navigate_to_frontier(best_frontier)
        else:
            self.get_logger().info('No more frontiers to explore.')

    def select_best_frontier(self, frontiers):
        best_score = -float('inf')
        best_frontier = None

        for frontier in frontiers:
            size = len(frontier)
            score = self.score_frontier(size)

            if score > best_score:
                best_score = score
                best_frontier = frontier

        return best_frontier


        
    def calculate_centroid(self, frontier):
        centroid_x = sum(p[0] for p in frontier) / len(frontier)
        centroid_y = sum(p[1] for p in frontier) / len(frontier)
        return Point(x=centroid_x, y=centroid_y)

  
    def score_frontier(self, size):
        # Adjust weights as needed. Higher `size_weight` favors larger frontiers.
        # Higher `distance_weight` favors more distant frontiers.
        size_weight = 1.0
        return size * size_weight

    def navigate_to_frontier(self, frontier):
        # Estimate centroid as the target position
        centroid_x = sum(p[0] for p in frontier) / len(frontier)
        centroid_y = sum(p[1] for p in frontier) / len(frontier)

        # Convert to world coordinates
        wx = centroid_x * self.map_data.info.resolution + self.map_data.info.origin.position.x
        wy = centroid_y * self.map_data.info.resolution + self.map_data.info.origin.position.y

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.map_data.header.frame_id
        goal_pose.pose.position = Point(x=wx, y=wy, z=0.0)
        goal_pose.pose.orientation.w = 1.0  # Neutral orientation

        self.get_logger().info(f'Navigating to new frontier at ({wx}, {wy})')

        self.action_client.wait_for_server()
        self.send_goal(goal_pose)

    def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info('Goal sent...')

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
