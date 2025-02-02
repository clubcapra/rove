import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class RadiationPublisher(Node):
    def __init__(self):
        super().__init__('radiation_publisher')
        self.publisher = self.create_publisher(Float32, '/dose_rate', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.publish_radiation_data)
        self.get_logger().info('Radiation publisher node has started.')
    
    def publish_radiation_data(self):
        msg = Float32()
        msg.data = 0.35 # à remplacer par la vraie valeur
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing radiation data: {msg.data}')
    
def main(args=None):
    rclpy.init(args=args)
    node = RadiationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

