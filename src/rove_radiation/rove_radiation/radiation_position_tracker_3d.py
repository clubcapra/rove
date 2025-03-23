import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import struct




class RadiationPositionTracker3D(Node):
    def __init__(self):
        super().__init__('radiation_position_tracker_3d')

        self.radiation_subscription = self.create_subscription(Float32, '/dose_rate', self.radiation_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odometry/local', self.localization_pose_callback, 10)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/radiation_point_cloud', 10)
        self.radiation_map = {} #pr stocker valeurs de radiation
        self.current_position = None

    def localization_pose_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    

    def radiation_callback(self, msg):
        if self.current_position is not None:
            self.radiation_map[self.current_position] = msg.data
            self.publish_point_cloud()
            #self.get_logger().info(f'position  = {self.current_position} , radiation = {msg.data}')

    def publish_point_cloud(self):
        rad_pos = []
        for(x,y,z), radiation in self.radiation_map.items():
            rad_pos.append(struct.pack('ffff', x, y, z, radiation)) #stocke sous forme de Float32
        
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        point_cloud_msg.header.frame_id = "map"
        point_cloud_msg.height = 1 # 1 car cest un nuage de pts organisé en ligne (pas en matrice)
        point_cloud_msg.width = len(rad_pos) # le nb tottal de pts
        point_cloud_msg.is_dense = False 
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 16  # 4 floats *4 octets =16 octets par point
        point_cloud_msg.row_step = point_cloud_msg.point_step * len(rad_pos) #la taille dune ligne complete
        point_cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=7, count=1), #coord en x (float32)
            PointField(name='y', offset=4, datatype=7, count=1), #coord en y (float32)
            PointField(name='z', offset=8, datatype=7, count=1), #coord en z (float32)
            PointField(name='intensity', offset=12, datatype=7, count=1), #valeur de radiation (float32)
        ]
        point_cloud_msg.data = b''.join(rad_pos) # ajouter les pts encodé dans le msg

        self.point_cloud_publisher.publish(point_cloud_msg)



def main(args=None):
    rclpy.init(args=args)
    node = RadiationPositionTracker3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
