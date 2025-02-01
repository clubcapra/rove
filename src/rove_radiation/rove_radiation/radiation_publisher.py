import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class RadiationPublisher(Node):
    def __init__(self):
        super().__init('radiation_publisher')
        self.publisher = self.create_publisher(Float32, '/dose_rate', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.publish_radiation_data)
    
    def publish_radiation_data(self):
        msg = Float32
        msg.data = 0.35 # à remplacer par la vraie valeur

