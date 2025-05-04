import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import struct
import math
import numpy as np
import cv2



class RadiationPositionTracker3D(Node):
    def __init__(self):
        super().__init__('radiation_position_tracker_3d')

        self.radiation_subscription = self.create_subscription(Float32, '/dose_rate', self.real_radiation_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odometry/local', self.localization_pose_callback, 10)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/radiation_point_cloud', 10)
        self.radiation_map = {} #pr stocker valeurs de radiation
        self.current_position = None
        #self.radius = 10  # Rayon de diffusion
        self.decay_factor = 0.05  # facteur de décroissance exponentiel

        self.map_size = 40
        self.radiation_map_local = np.zeros((self.map_size, self.map_size, self.map_size), dtype=np.float32)
        self.resolution = 0.05
        self.radius = self.map_size
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_radiation = None

        self.create_timer(0.5, self.update_publish_map)


    def real_radiation_callback(self, msg):
        self.current_radiation = msg.data

    def localization_pose_callback(self, msg):
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
    
    def update_publish_map(self):
        if self.current_radiation is None:
            return
        
        self.update_local_radiation_map()
        self.publish_point_cloud()

    def update_local_radiation_map(self):
        center_x = self.map_size // 2
        center_y = self.map_size // 2
        center_z = self.map_size // 2

        intensity = min(100, self.current_radiation * 100)

        for width in range(-self.radius, self.radius+1):
            for height in range(-self.radius, self.radius+1):
                for depth in range(-self.radius, self.radius+1):
                    coord_x = center_x + width
                    coord_y = center_y + height
                    coord_z = center_y + depth

                    if 0 <= coord_x < self.map_size and 0 < coord_z < self.map_size and 0 < coord_y < self.map_size: 
                        distance = math.sqrt(width **2 + height **2 + depth**2)
                        if distance <= self.radius:
                            attenuation = math.exp(-self.decay_factor * (distance//4))
                            self.radiation_map_local[coord_x, coord_y, coord_z] = intensity * attenuation
                            #self.get_logger().info(f'Update local radiation map : intensity  = {intensity} , position x = {coord_x}, position y = {coord_y}, position z = {coord_z}')
        self.radiation_map_local[center_x, center_y, center_z] = intensity

    

    # def radiation_callback(self, msg):
    #     """ if self.current_position is not None:
    #         self.radiation_map[self.current_position] = msg.data
    #         self.publish_point_cloud() """
    #         #self.get_logger().info(f'position  = {self.current_position} , radiation = {msg.data}')
    #     if self.current_position is not None:
    #         x0, y0, z0 = self.current_position
    #         intensity = min(100, msg.data * 100)  # Échelle similaire à la carte 2D
            
    #         # Mise à jour avec diffusion exponentielle
    #         for dx in range(-int(self.radius), int(self.radius) + 1):
    #             for dy in range(-int(self.radius), int(self.radius) + 1):
    #                 for dz in range(-int(self.radius), int(self.radius) + 1):
    #                     distance = math.sqrt(dx**2 + dy**2 + dz**2)
    #                     if distance <= self.radius:
    #                         attenuation = math.exp(-self.decay_factor * distance)
    #                         point = (x0 + dx, y0 + dy, z0 + dz)
    #                         self.radiation_map[point] = max(self.radiation_map.get(point, 0), intensity * attenuation)
            
    #     self.publish_point_cloud()


    def publish_point_cloud(self):
        rad_pos = []
        for x in range(0,self.map_size, 5):
            for y in range(0, self.map_size, 5):
                for z in range(0, self.map_size, 5):
                    intensity = self.radiation_map_local[x, y, z]
                    #self.get_logger().info(f'Publish poitn cloud : intensity  = {intensity} , position x = {x}, position y = {y}, position z = {z}')
                    real_x = (x - self.map_size // 2) * self.resolution + self.current_position.x
                    real_y = (y - self.map_size // 2) * self.resolution + self.current_position.y
                    real_z = (z - self.map_size // 2) * self.resolution + self.current_position.z

                    # Obtenir r,g,et b en float [0-1]
                    r, g, b = self.jet_colormap(intensity)

                    # Convertir en entiers [0-255]
                    r_int = int(r * 255)
                    g_int = int(g * 255)
                    b_int = int(b * 255)

                    # va pack RGB en un float32 (car c requis par rviz)
                    rgb = struct.unpack('f', struct.pack('I', (r_int << 16) | (g_int << 8) | b_int))[0]

                    rad_pos.append(struct.pack('ffff', real_x, real_y, real_z, rgb))        
        
        # for(x,y,z), radiation in self.radiation_map.items():
        #     rad_pos.append(struct.pack('ffff', x, y, z, radiation)) #stocke sous forme de Float32
        
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        point_cloud_msg.header.frame_id = "map"
        point_cloud_msg.height = 1 # 1 car c un nuage de pts organisé en ligne (pas en matrice)
        point_cloud_msg.width = len(rad_pos) # le nb tottal de pts
        point_cloud_msg.is_dense = False 
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 16  # 4 floats *4 octets =16 octets par point
        point_cloud_msg.row_step = point_cloud_msg.point_step * len(rad_pos) #la taille dune ligne complete
        point_cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=7, count=1), #coord en x (float32)
            PointField(name='y', offset=4, datatype=7, count=1), #coord en y (float32)
            PointField(name='z', offset=8, datatype=7, count=1), #coord en z (float32)
            PointField(name='rgb', offset=12, datatype=7, count=1),
        ]
        point_cloud_msg.data = b''.join(rad_pos) # ajouter les pts encodé dans le msg

        self.point_cloud_publisher.publish(point_cloud_msg)

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
    node = RadiationPositionTracker3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
