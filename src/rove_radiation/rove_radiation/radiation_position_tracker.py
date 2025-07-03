import datetime
from math import floor
import os
from typing import Optional

import matplotlib
from ament_index_python import get_package_share_directory
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
import matplotlib.cm as cm
from geometry_msgs.msg import PointStamped

class RadiationPositionTracker(Node):
    def __init__(self):
        super().__init__("radiation_position_tracker")
        self.max_rad_record = 0
        self.SIGMA = 20
        self.output_folder = os.path.abspath(os.path.join(
            get_package_share_directory("rove_radiation"), '../../../../output'
        ))

        self.radiation_subscription = self.create_subscription(
            Float32, "/dose_rate", self.radiation_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, "/odometry/local", self.localization_pose_callback, 10
        )

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )
        self.cost_map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )
        self.pub = self.create_publisher(PointStamped, 'radiation_points', 10)

        self.marker_publisher = self.create_publisher(Marker, "/radiation_marker", 10)

        self.declare_parameter("max_intensity", 100.0)
        self.max_intensity = (
            self.get_parameter("max_intensity").get_parameter_value().double_value
        )

        self.radiation_map_publisher = self.create_publisher(
            OccupancyGrid, "/radiation_map", 10
        )

        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_radiation: Optional[float] = None
        self.map: Optional[OccupancyGrid] = None
        self.obstacle_grid: Optional[OccupancyGrid] = None
        self.initialized = False

        self.create_timer(3, self.update_publish_map)

    def radiation_callback(self, msg):
        """Stocke la dernière valeur mesurée de radiation"""
        self.current_radiation = msg.data
        if self.current_radiation > self.max_rad_record: 
            self.max_rad_record = self.current_radiation
        # if self.current_radiation > self.max_intensity: 
        #     self.max_intensity = self.current_radiation
        self.send_point(self.current_position.x, self.current_position.y, self.current_radiation)

    def localization_pose_callback(self, msg):
        """Met à jour la position du robot. Cette callback est utilisée pour Odometry ou AMCL selon votre configuration"""
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z

    def map_callback(self, msg):
        """
        Reçoit nouvelle carte SLAM (OccupancyGrid : self.obstacle_grid).
        - Conserve l'historique (au niveau des dimensions) de la carte de radiation au fur et à mesure que sa taille change
        - Reconstruit la nouvelel données en recollant l'ancien contenu, initialise à une valeur de -1 ailleurs
        """
        # 1) mettre à jour l'obstacle_grid
        self.obstacle_grid = msg

        # 2)mémoriser l'ancienne taille et l'ancien data
        if self.map is not None:
            old_w = self.map.info.width
            old_h = self.map.info.height
            old_data = np.array(self.map.data, dtype=np.int8).reshape((old_h, old_w))
        else:
            old_w = old_h = 0
            old_data = None

        # 3)(Re)création de la grille de radiation si première fois
        if self.map is None:
            self.map = OccupancyGrid()

        # 4)Mise à jour systématique de taille, origine,resolution et header
        self.map.header = msg.header
        self.map.info = msg.info

        # 5)Dimensions de la nouvelle grille
        new_w = msg.info.width
        new_h = msg.info.height
        total_cells = new_w * new_h

        if not self.initialized:
            self.map.data = [-1] * total_cells
            self.initialized = True

        # 6)reconstruire le tableau avec le data :
        #    - Copie zone déjà mesurée
        #    - Initialise à -1 les cellules "neuves" (non mesurées)
        grid = np.full((new_h, new_w), -1, dtype=np.int8)
        if old_data is not None:
            H = min(old_h, new_h)
            W = min(old_w, new_w)
            grid[:H, :W] = old_data[:H, :W]
        self.map.data = grid.flatten().tolist()

        # 7)
        self.update_publish_map()

    def update_publish_map(self):
        """
        Intègre la dernière mesure de radiation à la grille interne,
        publie l'occupancy grid représentant la grille/carte de radiation sur le topic /radiation_map
        """
        if self.map is not None and self.current_radiation is not None:
            map_origin = self.map.info.origin.position
            map_resolution = round(self.map.info.resolution, 4)
            map_width = self.map.info.width
            map_height = self.map.info.height

            grid_x = floor((self.current_position.x - map_origin.x) / map_resolution)
            grid_y = floor((self.current_position.y - map_origin.y) / map_resolution)
            grid_index = grid_y * map_width + grid_x

            if 0 <= grid_index < len(self.map.data):
                self.map.header.stamp = self.get_clock().now().to_msg()
                self.map.data = list(self.map.data)

                value = int(round(100 * (self.current_radiation / self.max_intensity)))
                value = max(0, min(100, value))  # clamp entre [0, 100]
                self.map.data[grid_index] = value

                
                self.generate_and_save_image()
                normed = self.generate_heat_map()
                
                rad_map = OccupancyGrid()
                int_data = (normed * 100).astype(np.int8).flatten().tolist()
                rad_map.data = int_data
                rad_map.header = self.map.header
                rad_map.info = self.map.info
                
                self.radiation_map_publisher.publish(rad_map)

    def generate_heat_map(self):
        og = self.obstacle_grid.info
        width = og.width
        height = og.height
    
        obstacle_data = np.array(self.obstacle_grid.data, dtype=np.int8).reshape((height, width))
        radiation_data = np.array(self.map.data, dtype=np.float32).reshape((height, width))
    
        image = np.full((height, width, 3), fill_value=0.85, dtype=np.float32)
        obstacle_mask = obstacle_data >= 50
        image[obstacle_mask] = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    
        measured_indices = np.argwhere(radiation_data > 0)
        p = 8  # power for smooth max, tune this
    
        accum = np.zeros((height, width), dtype=np.float32)
        
        for i, j in measured_indices:
            temp = np.zeros((height, width), dtype=np.float32)
            intensity = radiation_data[i, j]
            temp[i, j] = intensity
            blurred = gaussian_filter(temp, sigma=self.SIGMA)
            accum += blurred**p  # accumulate powers
    
        heatmap = accum**(1/p)  # power mean
    
        # Normalize for display
        normed = heatmap / np.max(heatmap) if np.max(heatmap) > 0 else heatmap
    
        colored_heatmap = cm.jet(normed)[..., :3]
    
        alpha = 0.6
        image = (1 - alpha) * image + alpha * colored_heatmap
    
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.imshow(image)
    
        cbar = fig.colorbar(
            cm.ScalarMappable(norm=matplotlib.colors.Normalize(vmin=0, vmax=self.max_rad_record), cmap='jet'),
            ax=ax
        )
        cbar.set_label("Radiation intensity [µSv]", fontsize=12)
        ax.set_xticks([])
        ax.set_yticks([])
        
        plt.title("Enrich 2025 Capra Radiation heatmap", fontsize=16, fontweight='bold', y=0.95)

        plt.savefig(os.path.join(self.output_folder, "radiation_heatmap.pdf"), bbox_inches='tight')
        plt.close()
    
        return normed



    def generate_and_save_image(self):
        """
        Génère une image PDF de la carte fusionnant l'occupancy grid venant du SLAM et celui venant de la radiation,
        avec une color map 'jet' pour les mesures de radiation qui > 0
        """
        og = self.obstacle_grid.info  # OccupancyGrid obstacle
        rg = self.map.info  # OccupancyGrid radiation

        # parfois les asserts sont faux donc ca crash...
        """ assert og.width == rg.width, "le Width des deux grilles est différent"
        assert og.height == rg.height, "le Height des deux grilles différent"
        assert np.isclose(og.resolution, rg.resolution), "la Resolution différente" """

        width = og.width
        height = og.height

        obstacle_data = np.array(self.obstacle_grid.data, dtype=np.int8).reshape(
            (height, width)
        )
        radiation_data = np.array(self.map.data, dtype=np.int8).reshape((height, width))

        # fond gris clair + murs noirs
        image = np.full((height, width, 3), fill_value=0.85, dtype=np.float32)
        obstacle_mask = obstacle_data >= 50  # revoir
        image[obstacle_mask] = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # coloration des cellules mesurées qui ont une valeur de radiation > 0
        measured_indices = np.argwhere(radiation_data > 0)
        rayon = 3
        cmap = plt.get_cmap("jet")
        for (i, j) in measured_indices:
            raw = int(radiation_data[i, j])
            v = raw / 100.0
            if raw >= 100:
                rgb = (1.0, 0.0, 0.0)  # rouge pour quand radiation est a 100 %
            else:
                rgb = cmap(v)[:3]  # sinon bleu a rouge

            imin = max(i - rayon, 0)
            imax = min(i + rayon, height - 1)
            jmin = max(j - rayon, 0)
            jmax = min(j + rayon, width - 1)

            image[imin : imax + 1, jmin : jmax + 1, :] = rgb
        # self.get_logger().info(str(radiation_data))
        # heatmap, xedges, yedges = np.histogram2d(measured_indices[:,0],measured_indices[:,1], bins=(width, height))
        # extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
        # config figure et grille
        zoom = 2  # 2×2 pixel par cellule
        dpi = 100

        fig_w = (width * zoom) / dpi
        fig_h = (height * zoom) / dpi

        fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)

        # Afficher limage
        ax.imshow(image, origin="lower", cmap='hot', interpolation="nearest")
        # ax.imshow(heatmap.T, extent=extent, origin="lower")

        # Tracer la grille
        ax.set_xticks(np.arange(-0.5, width, 1), minor=False)
        ax.set_yticks(np.arange(-0.5, height, 1), minor=False)
        ax.grid(which="both", color="black", linestyle="-", linewidth=0.2)

        ax.axis("off")

        cbar = fig.colorbar(
            cm.ScalarMappable(norm=matplotlib.colors.Normalize(vmin=0, vmax=self.max_intensity), cmap='jet'),
            ax=ax
        )
        cbar.set_label("Radiation intensity [µSv]", fontsize=12)
        
        ax.set_xticks([])
        ax.set_yticks([])
        plt.title("Enrich 2025 Radiation Map of Recorded Data Points", fontsize=16, fontweight='bold')
        
        # save le pdf
        sortie = os.path.join(self.output_folder, "radiation_map_points.pdf")  # pt etre mettre ca en param en cli
        fig.savefig(sortie, bbox_inches="tight", pad_inches=0)
        plt.close(fig)

    def send_point(self, x, y, radiation_level, frame='map'):
        msg = PointStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = radiation_level
        self.pub.publish(msg)
        self.get_logger().info(f'Published radiation point at ({x:.2f}, {y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = RadiationPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
