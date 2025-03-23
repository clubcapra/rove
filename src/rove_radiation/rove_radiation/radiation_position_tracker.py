import math
import cv2
import numpy as np
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge




class RadiationPositionTracker(Node):
    def __init__(self):
        super().__init__('radiation_position_tracker')
        
        self.radiation_subscription = self.create_subscription(Float32, '/dose_rate', self.radiation_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odometry/local', self.localization_pose_callback, 10)
        self.ocuppancy_grid_publisher = self.create_publisher(OccupancyGrid, '/radiation_occupancy_grid', 10)
        self.ocuppancy_grid_static_publisher = self.create_publisher(OccupancyGrid, '/radiation_occupancy_grid_static', 10)
        self.markers_publisher = self.create_publisher(MarkerArray, '/radiation_markers', 10)

        #self.map_subscription  = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        """ self.radiation_position_publisher = self.create_publisher(Float32, '/dose_rate', 10)
        self.radiation_map_publisher = self.create_publisher(OccupancyGrid, '/radiation_map', 10)
        self.markers_publisher = self.create_publisher(MarkerArray, '/radiation_markers', 10) """

        self.heatmap_publisher = self.create_publisher(Image, "/radiation_map_image", 10)

        self.map_size = 40  #taille de la carte locale (10x10 cellules) (carte locale = champ de vision de la carte de radioactivité du robot...)
        self.resolution = 0.1  # 10 cm par cellule
        self.radiation_map_local = np.zeros((self.map_size, self.map_size), dtype=np.float32)
        self.radiation_map_static = np.zeros((self.map_size, self.map_size), dtype=np.float32)
        self.radius = self.map_size
        self.decay_factor = 0.05
        self.hotspot_seuil = 60

        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_radiation = None

        self.bridge = CvBridge()
        self.create_timer(0.5, self.update_publish_map)

        #self.markers_array = MarkerArray()
               
    def radiation_callback(self, msg):
        self.current_radiation = msg.data
    
    def localization_pose_callback(self, msg):
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z

    def update_publish_map(self):
        if self.current_radiation is None:
            return  
        #met a jour la carte locale avec la radiation
        self.update_local_radiation_map()

        self.publish_occupancy_grid(self.radiation_map_local, "local" ) 
        self.publish_markers(self.radiation_map_local) 

        #publish la heatmap en tant qu'image
        self.publish_radiation_heatmap(self.radiation_map_local.copy())
    
    def update_local_radiation_map(self):
        #centre la carte pr/r au robot...
        center_x = self.map_size // 2
        center_y = self.map_size // 2

        #converti la radiation en intensité (ex. sensor detcte 0.7 alors 70... car probleme quand pas un int donc *100 résout ca)
        intensity = min(100, self.current_radiation * 100)

        for width in range(-self.radius, self.radius+1):
            for height in range(-self.radius, self.radius+1):
                coord_x = center_x + width
                coord_y = center_y + height

                if 0 <= coord_x < self.map_size and 0 < coord_y < self.map_size:
                    distance = math.sqrt(width **2 + height **2)
                    if distance <= self.radius:
                        attenuation = math.exp(-self.decay_factor * (distance//4))
                        self.radiation_map_local[coord_x, coord_y] = intensity * attenuation
        #ajouter radiation détecté à la position centrale
        self.radiation_map_local[center_y, center_x] = intensity
        if(self.radiation_map_local[center_y, center_x] >= self.hotspot_seuil):
            self.build_radiation_map_static()
            #self.get_logger().info(f'static... radiation = {self.radiation_map_local[center_y, center_x]}')

    def build_radiation_map_static(self):
        self.radiation_map_static = self.radiation_map_local
        self.publish_occupancy_grid(self.radiation_map_static, "static")  
        self.publish_markers(self.radiation_map_static) 

        #publish la heatmap en tant qu'image
        self.publish_radiation_heatmap(self.radiation_map_local.copy())

    def publish_radiation_heatmap(self, heatmap):
        #heatmap = self.radiation_map_local.copy()

        heatmap = np.clip(heatmap, 0, 100) #à retester mais cette ligne devrait ne plus etre pertinente
        heatmap = (heatmap / 100) * 255 #transforme entre des valeur de 0 à 255 ârce que Opencv oblige ca
        heatmap = heatmap.astype(np.uint8) #converti en entier car cest demander pour affichage d,image...

        #apppliquer de la couleur pour plus de réaliste... un peu comme une heatmap 
        heatmap_colored = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
        """ Légende des couleurs de  COLORMAP_JET:
        Bleu = faible radiation
        Vert = radiation moyenne
        Rouge/jaune = forte radiation """

        #conversion en un msg ros
        heatmap_msg = self.bridge.cv2_to_imgmsg(heatmap_colored, encoding="bgr8")
        heatmap_msg.header.stamp = self.get_clock().now().to_msg()
        heatmap_msg.header.frame_id = "odom"  

        self.heatmap_publisher.publish(heatmap_msg)

    def publish_occupancy_grid(self, radiation_map, map_type):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.map_size
        grid_msg.info.height = self.map_size
        grid_msg.info.origin.position.x = self.current_position.x - (self.map_size * self.resolution) / 2
        grid_msg.info.origin.position.y = self.current_position.y - (self.map_size * self.resolution) / 2
        grid_msg.info.origin.orientation.w = 1.0

        grid_data = np.clip(radiation_map, 0, 100)  
        grid_data = (grid_data / 100) * 100  
        grid_data = grid_data.astype(np.int8).flatten().tolist()

        grid_msg.data = grid_data
        if map_type == "local":
            self.ocuppancy_grid_publisher.publish(grid_msg)
        elif map_type == "static":
            self.ocuppancy_grid_static_publisher.publish(grid_msg)
    
    
    def publish_markers(self,radiation_map):
        markers_msg = MarkerArray()
        marker_id = 0

        for i in range(self.map_size):
            for j in range(self.map_size):
                intensity = radiation_map[i, j]
                 
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "map"
                marker.id = marker_id
                marker_id += 1

                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = self.resolution

                marker.pose.position.x = self.current_position.x + (i - self.map_size//2) * self.resolution
                marker.pose.position.y = self.current_position.y + (j - self.map_size//2) * self.resolution
                marker.pose.position.z = 0.0

                r, g, b = self.jet_colormap(intensity)
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = 0.8
                """ marker.color.a = 1.0
                marker.color.r = min(1.0, intensity / 100)
                marker.color.g = 1.0 - min(1.0, intensity / 100)
                marker.color.b = 0.0 """

                markers_msg.markers.append(marker)

        self.markers_publisher.publish(markers_msg)

    def jet_colormap(self, value, vmin=0, vmax=100):
        """
        Convertit une valeur d'intensité en une couleur JET.
        value : Intensité (entre vmin et vmax).
        """
        #Normlaiser entre 0 et 255 (pour OpenCV )
        normalized = np.clip((value - vmin) / (vmax - vmin), 0, 1) * 255
        colormap = cv2.applyColorMap(np.uint8([[normalized]]), cv2.COLORMAP_JET)
        b, g, r = colormap[0, 0]  #OpenCV donne en BGR, faut donc inverser

        return r / 255.0, g / 255.0, b / 255.0  #Normaliser entre 0 et 1


        

def main(args=None):
    rclpy.init(args=args)
    node = RadiationPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
