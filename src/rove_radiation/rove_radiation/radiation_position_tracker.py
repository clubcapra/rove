import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Point


class RadiationPositionTracker(Node):
    def __init__(self):
        super().__init__('radiation_position_tracker')
        
        self.radiation_subscription = self.create_subscription(Float32, '/dose_rate', self.radiation_callback, 10)
        self.odom_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.localization_pose_callback, 10)

        self.radiation_position_publisher = self.create_publisher(Float32, '/dose_rate', 10)

        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_radiation 
        
    def radiation_callback(self, msg):
        self.get_logger().info(f'radiation detected: {msg.data}')
        self.current_radiation = msg.data

        if self.current_position is not None and self.current_radiation is not None:
            pass

    
    def localization_pose_callback(self, msg):
        # Met à jour la position du robot
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        self.get_logger().info(f'current robot position: x : {self.current_position.x}, y : {self.current_position.y}, z : {self.current_position.z}')
    
    
def main(args=None):
    rclpy.init(args=args)
    node = RadiationPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
