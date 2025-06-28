import io
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import csv

class RadiationMapDataViewer(Node):
    def __init__(self):
        super().__init__("radiation_map_data_viewer")
        self.subscription = self.create_subscription(
            OccupancyGrid, "/radiation_map", self.map_callback, 10
        )

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        # Reconstruction en grille 2D
        grid_2d = [data[i * width : (i + 1) * width] for i in range(height)]

        # Sauvegarde dans un fichier CSV
        # with open("../radiation_map.csv", "+w", newline="") as f:
        #     writer = csv.writer(f)
        #     for row in grid_2d:
        #         writer.writerow(row)


def main():
    rclpy.init()
    node = RadiationMapDataViewer()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
