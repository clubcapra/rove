import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid


class RadiationPositionTracker(Node):
    def __init__(self):
        super().__init__('radiation_position_tracker')
        
        self.radiation_subscription = self.create_subscription(Float32, '/dose_rate', self.radiation_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odometry/local', self.localization_pose_callback, 10)
        self.map_subscription  = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.radiation_position_publisher = self.create_publisher(Float32, '/dose_rate', 10)
        self.radiation_map_publisher = self.create_publisher(OccupancyGrid, '/radiation_map', 10)

        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_radiation = None
        self.map = None

        self.create_timer(0.5, self.update_publish_map)
        
    def radiation_callback(self, msg):
        self.current_radiation = msg.data
    
    def localization_pose_callback(self, msg):
        # Update robot's position
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        
    def map_callback(self, msg):
        if self.map is None: 
            self.map = OccupancyGrid()
            self.map.header = msg.header
            self.map.info = msg.info
            self.map.data = [-1] * (msg.info.width * msg.info.height)
            self.map.data = list(msg.data)

        self.update_publish_map()

    def update_publish_map(self):
        if self.map is not None and self.current_radiation is not None :
            map_origin = self.map.info.origin.position
            map_resolution = self.map.info.resolution
            map_width = self.map.info.width

            grid_x = int((self.current_position.x - map_origin.x)/map_resolution)
            grid_y = int((self.current_position.y - map_origin.y)/map_resolution)
            grid_index = grid_y * map_width + grid_x

            if 0 <= grid_index < len(self.map.data):
                self.map.header.stamp = self.get_clock().now().to_msg()
                self.map.data = list(self.map.data) 
                self.map.data[grid_index] = int(round(self.current_radiation * 100))
                self.radiation_map_publisher.publish(self.map)

def main(args=None):
    rclpy.init(args=args)
    node = RadiationPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
