from math import floor
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


class RadiationPositionTracker(Node):
    def __init__(self):
        super().__init__('radiation_position_tracker')
        
        self.radiation_subscription = self.create_subscription(Float32, '/dose_rate', self.radiation_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odometry/local', self.localization_pose_callback, 10)
        
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.map_subscription = self.create_subscription(OccupancyGrid,'/map',self.map_callback, map_qos)
        
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.localization_pose_callback,
            QoSProfile(depth=10)
        )

        #self.map_subscription  = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        #self.get_logger().info("Subscribed to /map")
        marker_qos = QoSProfile(depth=100,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.marker_publisher = self.create_publisher(Marker, '/radiation_marker',10)


        self.declare_parameter("max_intensity", 100.0)
        self.max_intensity = self.get_parameter("max_intensity").get_parameter_value().double_value
        #self.get_logger().info(f"Max intensity set to: {self.max_intensity}")


        #self.radiation_position_publisher = self.create_publisher(Float32, '/dose_rate', 10)
        self.radiation_map_publisher = self.create_publisher(OccupancyGrid, '/radiation_map', 10)

        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_radiation = None
        self.map = None
        self.obstacle_grid = None  
        self.marker_id = 0

        #test pour positions ecrasement donnée radiation
        """ self.test_phase = 1
        self.create_timer(10, self.update_test_position, callback_group=None)
        self.force_position_phase() """

        self.create_timer(3, self.update_publish_map)
        
    def radiation_callback(self, msg):
        self.current_radiation = msg.data
        #self.get_logger().info(f'Received radiation data: {msg.data}')

    #debut méthode pour test positions ecrasement donnée radiation
    def force_position_phase(self):
        if self.test_phase == 1:
            self.current_position.x = 2.00
            self.current_position.y = 1.00
            #self.get_logger().info(f'Phase 1 : position forcée à ({self.current_position.x}, {self.current_position.y})')
        elif self.test_phase == 2:
            self.current_position.x = 2.03
            self.current_position.y = 1.03
            #self.get_logger().info(f'Phase 2 : position forcée à ({self.current_position.x}, {self.current_position.y})')
    
    def update_test_position(self):
        if self.test_phase == 1:
            self.test_phase = 2
            self.force_position_phase()  
    #fin méthode pour test positions ecrasement donnée radiation

    
    def localization_pose_callback(self, msg):
        # Update robot' position
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z

        """ self.current_position.x = 2.0
        self.current_position.y = 1.0
        self.current_position.z = 0.0 """

        
    def map_callback(self, msg):
        #self.get_logger().info("Map callback called")
        #self.get_logger().info(f"Received map with timestamp: {msg.header.stamp}")
        self.obstacle_grid = msg
        if self.map is None: 
            #self.get_logger().info('self.map is NONE!!!!!')
            self.map = OccupancyGrid()
            self.map.header = msg.header
            self.map.info = msg.info
            self.map.data = [-1] * (msg.info.width * msg.info.height)
            self.map.data = list(self.map.data)
        self.update_publish_map()

    def update_publish_map(self):
        #self.get_logger().info(f'Is Map none : {self.map is None}')
        #self.get_logger().info(f'Is Radiation none : {self.current_radiation is None}')
        if self.map is not None and self.current_radiation is not None :
            map_origin = self.map.info.origin.position
            #map_resolution = 0.1
            map_resolution = round(self.map.info.resolution, 4)
            map_width = self.map.info.width
            map_height = self.map.info.height
            #self.get_logger().info(f"Map Resolution: {self.map.info.resolution}")

            grid_x = floor((self.current_position.x - map_origin.x)/map_resolution)
            grid_y = floor((self.current_position.y - map_origin.y)/map_resolution)
            grid_index = grid_y * map_width + grid_x

            """ self.get_logger().info(
            f"Position: ({self.current_position.x:.2f}, {self.current_position.y:.2f}) "
            f"--> index: {grid_index} (grid_x: {grid_x}, grid_y: {grid_y})"
            f"--> data : {self.current_radiation}"
            ) """

            if 0 <= grid_index < len(self.map.data):
                self.map.header.stamp = self.get_clock().now().to_msg()
                self.map.data = list(self.map.data) 
                #self.map.data[grid_index] = int(round(self.current_radiation * 100))

                value = int(round(100 * (self.current_radiation / self.max_intensity)))
                value = max(0, min(100, value))  # clamp entre [0, 100]
                self.map.data[grid_index] = value
                
                #self.get_logger().info(f'index : {grid_index} ; data : {self.map.data[grid_index]}')
                #self.get_logger().info(f'value : {value} ; current_radiation : {self.current_radiation} ; max_intensity : {self.max_intensity} resultat calcul : {100 * (self.current_radiation / self.max_intensity)}')

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

                self.get_logger().info(f"Obstacle Map Width: {self.obstacle_grid.info.width}")
                self.get_logger().info(f"Radiation Map Width: {self.map.info.width}")
                self.generate_and_save_image()
    

    
    def generate_and_save_image(self):
        og = self.obstacle_grid.info # OccupancyGrid obstacle
        rg = self.map.info # OccupancyGrid radiation

        #parfois les asserts sont faux donc ca crash... 
        assert og.width == rg.width, "le Width des deux grilles est différent"
        assert og.height == rg.height, "le Height des deux grilles différent"
        assert np.isclose(og.resolution, rg.resolution), "la Resolution différente"

        width = og.width
        height = og.height

        obstacle_data = np.array(self.obstacle_grid.data, dtype=np.int8).reshape((height, width))
        radiation_data = np.array(self.map.data, dtype=np.int8).reshape((height, width))

        image = np.full((height, width, 3), fill_value=0.85, dtype=np.float32)
        obstacle_mask = (obstacle_data >= 50)  # revoir car je pourrais mettre ou ==100
        image[obstacle_mask] = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        """ norm_map = np.zeros((height, width), dtype=np.float32)
        valid_mask = (radiation_data >= 0)
        norm_map[valid_mask] = radiation_data[valid_mask] / 100.0 """

        measured_indices = np.argwhere(radiation_data > 0)
        rayon = 1

        cmap = plt.get_cmap('jet')
        for (i, j) in measured_indices:
            raw = int(radiation_data[i, j])
            v   = raw / 100.0
            if raw >= 100:
                rgb = (1.0, 0.0, 0.0) #rouge pour quand radiation est a 100 %
            else:
                rgb = cmap(v)[:3] #sinon bleu a rouge

            imin = max(i - rayon, 0)
            imax = min(i + rayon, height - 1)
            jmin = max(j - rayon, 0)
            jmax = min(j + rayon, width  - 1)

            image[imin:imax+1, jmin:jmax+1, :] = rgb


        zoom = 2 #2×2 pixel par cellule
        dpi  = 100

        fig_w = (width  * zoom) / dpi  
        fig_h = (height * zoom) / dpi  

        fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)

        # Afficher limage 
        ax.imshow(image, origin='lower', interpolation='nearest')

         #Tracer la grille
        ax.set_xticks(np.arange(-0.5, width, 1), minor=False)
        ax.set_yticks(np.arange(-0.5, height, 1), minor=False)
        ax.grid(which='both', color='black', linestyle='-', linewidth=0.2)

        ax.axis('off')

        # save le pdf
        sortie = '/home/janice/radiation_map.pdf' #pt etre mettre ca en param en cli
        fig.savefig(sortie, bbox_inches='tight', pad_inches=0)
        plt.close(fig)

        self.get_logger().info(f"le pdf se sauvegarde : {sortie} "
                            f"(dimensions {fig_w*dpi:.0f}×{fig_h*dpi:.0f} pixel)")


def main(args=None):
    rclpy.init(args=args)
    node = RadiationPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

