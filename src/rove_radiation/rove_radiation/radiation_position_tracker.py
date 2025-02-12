import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid


class RadiationPositionTracker(Node):
    def __init__(self):
        super().__init__('radiation_position_tracker')
        
        self.radiation_subscription = self.create_subscription(Float32, '/dose_rate', self.radiation_callback, 10)
        self.odom_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.localization_pose_callback, 10)
        self.map_subscription  = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.radiation_position_publisher = self.create_publisher(Float32, '/dose_rate', 10)
        self.radiation_map_publisher = self.create_publisher(OccupancyGrid, '/radiation_map', 10)

        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_radiation = None
        self.map = None

        self.create_timer(1.0, self.update_publish_map)

        
    def radiation_callback(self, msg):
        #self.get_logger().info(f'radiation detected: {msg.data}')
        self.current_radiation = msg.data

        if self.current_position is not None and self.current_radiation is not None:
            pass

    
    def localization_pose_callback(self, msg):
        # Met à jour la position du robot
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        #self.get_logger().info(f'current robot position: x : {self.current_position.x}, y : {self.current_position.y}, z : {self.current_position.z}')
    
    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info(f"Type of map data: {type(self.map.data)}, Example value: {self.map.data[0]}")

        """ if self.map and self.map.data:
            self.get_logger().info(f'Map width x height: {msg.info.width} x {msg.info.height}, Map resolution: {msg.info.resolution}')
            self.get_logger().info(f"Map origin: x: {msg.info.origin.position.x}, y: {msg.info.origin.position.y}")
            self.get_logger().info(f"Map data size: {len(self.map.data)}")
        else:
            self.get_logger().error("Received invalid map data!") """

        #self.get_logger().info(f'Map width x height : {msg.info.width} x {msg.info.height}, Map resolution : {msg.info.resolution}')
        self.update_publish_map()

    def update_publish_map(self):
        #self.get_logger().info("update_publish_map() called")

        """ if self.map is None or self.current_radiation is None:
            self.get_logger().info("Waiting for map and radiation data...")
            if self.map is not None:
                self.get_logger().info(f"Map data size: {len(self.map.data)}")
            self.get_logger().info(f"Radiation: {self.current_radiation}, Position: {self.current_position.x}, {self.current_position.y}")
            return None
         """
        
        if self.map is not None and self.current_radiation is not None :
            map_origin = self.map.info.origin.position
            map_resolution = self.map.info.resolution
            map_width = self.map.info.width

            grid_x = int((self.current_position.x - map_origin.x)/map_resolution)
            grid_y = int((self.current_position.y - map_origin.y)/map_resolution)
            grid_index = grid_y * map_width + grid_x

            self.get_logger().info(f"Map origin: {map_origin.x}, {map_origin.y}")
            self.get_logger().info(f"Calculated grid_x: {grid_x}, grid_y: {grid_y}, grid_index: {grid_index}")


            if 0 <= grid_index < len(self.map.data):
                self.map.header.stamp = self.get_clock().now().to_msg()
                self.map.data = list(self.map.data) 
                self.map.data[grid_index] = int(round(self.current_radiation * 100))
                self.radiation_map_publisher.publish(self.map)

 
                # self.get_logger().info(f"grid_x: {grid_x}, grid_y: {grid_y}, grid_index: {grid_index}, map_size: {len(self.map.data)}")
                # updated_map = OccupancyGrid()
                # updated_map.header.stamp = self.get_clock().now().to_msg() # to synchronize with rviz... (in case)
                # updated_map.header.frame_id = "map"
                # updated_map.info = self.map.info
                # updated_map.data = list(self.map.data)  
                #self.get_logger().info(f"Initial memory address of updated_map.data: {id(updated_map.data)}")
                # updated_map.data[grid_index] = int(round(self.current_radiation * 100))
                # self.get_logger().info(f"After update: Value at grid_index {grid_index}: {updated_map.data[grid_index]}")
                # #updated_map.data[grid_index] = round(self.current_radiation * 100)
                # self.get_logger().info(f"Before publishing: Value at grid_index {grid_index}: {updated_map.data[grid_index]}")

                # self.get_logger().info(f"Final map data before publishing (sample): {list(updated_map.data[122450:122460])}")
                # updated_map.data = list(updated_map.data)  # Convertir en liste pour éviter les problèmes de références
                # #self.get_logger().info(f"Final memory address of updated_map.data: {id(updated_map.data)}")

                # self.radiation_map_publisher.publish(updated_map)
                # self.get_logger().info(f"Updating map at grid_index {grid_index} with radiation value {self.current_radiation}")
                # self.get_logger().info(f"Value at grid index {grid_index}: {updated_map.data[grid_index]}")
                #self.get_logger().info(f"Publishing updated radiation map at position: {self.current_position}")

        
    
def main(args=None):
    rclpy.init(args=args)
    node = RadiationPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
