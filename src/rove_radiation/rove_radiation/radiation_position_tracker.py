import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy



class RadiationPositionTracker(Node):
    def __init__(self):
        super().__init__('radiation_position_tracker')
        
        self.radiation_subscription = self.create_subscription(Float32, '/dose_rate', self.radiation_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odometry/local', self.localization_pose_callback, 10)
        
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.map_subscription = self.create_subscription(OccupancyGrid,'/map',self.map_callback,map_qos)
        
        #self.map_subscription  = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.get_logger().info("Subscribed to /map")
        self.marker_publisher = self.create_publisher(Marker, '/radiation_marker', 10)

        self.declare_parameter("max_intensity", 20.0)
        self.max_intensity = self.get_parameter("max_intensity").get_parameter_value().double_value
        #self.get_logger().info(f"Max intensity set to: {self.max_intensity}")


        #self.radiation_position_publisher = self.create_publisher(Float32, '/dose_rate', 10)
        self.radiation_map_publisher = self.create_publisher(OccupancyGrid, '/radiation_map', 10)

        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_radiation = None
        self.map = None
        self.marker_id = 0


        self.create_timer(3, self.update_publish_map)
        
    def radiation_callback(self, msg):
        self.current_radiation = msg.data
        #self.get_logger().info(f'Received radiation data: {msg.data}')

    
    def localization_pose_callback(self, msg):
        # Update robot' position
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        
    def map_callback(self, msg):
        self.get_logger().info("Map callback called")
        self.get_logger().info(f"Received map with timestamp: {msg.header.stamp}")
        if self.map is None: 
            self.map = OccupancyGrid()
            self.map.header = msg.header
            self.map.info = msg.info
            self.map.data = [-1] * (msg.info.width * msg.info.height)
            self.map.data = list(msg.data)

        self.update_publish_map()

    def update_publish_map(self):
        self.get_logger().info(f'Is Map none : {self.map is None}')
        self.get_logger().info(f'Is Radiation none : {self.current_radiation is None}')
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
                #self.map.data[grid_index] = int(round(self.current_radiation * 100))

                value = int(round(100 * (self.current_radiation / self.max_intensity)))
                value = max(0, min(100, value))  # clamp entre [0, 100]
                self.map.data[grid_index] = value

                self.radiation_map_publisher.publish(self.map)

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "radiation"
                marker.id = self.marker_id
                self.marker_id += 1
                marker.type = Marker.SPHERE  
                marker.action = Marker.ADD

                marker.pose.position = self.current_position
                marker.pose.orientation.w = 1.0 

                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.01  

                # Heatmap color (blue to red)
                #intensity = max(0.0, min(self.current_radiation / 2.0, 1.0))  
                intensity = max(0.0, min(self.current_radiation / self.max_intensity, 1.0))
                #self.get_logger().info(f"Calculated Intensity: {intensity}")


                marker.color.r = intensity
                marker.color.g = 0.0
                marker.color.b = 1.0 - intensity
                marker.color.a = 1.0  

                self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RadiationPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
