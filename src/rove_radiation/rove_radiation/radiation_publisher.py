import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class RadiationPublisher(Node):
    def __init__(self):
        super().__init__('radiation_publisher')
        self.source_x = 4.7071488
        self.source_y = 0.0105071
        """ self.source_x = 1.796319
        self.source_y = -4.083129e-05
        self.source_z = 1 """

        # self.source2_x = -6.288366
        # self.source2_y = -0.0121495
        # self.source3_x = -4.92
        # self.source3_y = -2.01

        self.max_radiation = 0.87  #valeur maximum que la radiation peut avoir... donc cest la valeur de radiation quand on est pile poil sur le x et y de la radiation
        self.decay_factor = 1   #Plus la valeur est garnde plus ca descend vite quand on seloinge

        self.odom_subscription = self.create_subscription(Odometry, '/odometry/local', self.odom_callback, 10)
        self.radiation_publisher = self.create_publisher(Float32, '/dose_rate', 10)
        #self.get_logger().info('Radiation publisher node has started.')

        #self.marker_publisher = self.create_publisher(Marker, "/radiation_sources", 10)
        self.publish_radiation_markers()
    
    def odom_callback(self, msg):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        #Calcul pour calculer distance entre robot et source de radiation (distance euclidienne)
        distance = math.sqrt((robot_x - self.source_x) ** 2 + (robot_y - self.source_y) ** 2)
        # distance2 = math.sqrt((robot_x - self.source2_x) ** 2 + (robot_y - self.source2_y) ** 2)
        # distance3 = math.sqrt((robot_x - self.source3_x) ** 2 + (robot_y - self.source3_y) ** 2)

        #formule de decroissance exponetielle selon distance
        radiation_value = self.max_radiation * math.exp(-self.decay_factor * distance)
        # radiation_value2 = self.max_radiation * math.exp(-self.decay_factor * distance2)
        # radiation_value3 = self.max_radiation * math.exp(-self.decay_factor * distance3)


        # Publier la valeur de radiation
        msg_radiation = Float32()
        msg_radiation.data = radiation_value
        #msg_radiation.data = radiation_value + radiation_value2 + radiation_value3
        self.radiation_publisher.publish(msg_radiation)

    def publish_radiation_markers(self):
        #FONCTIONNE PAS""" JVOIS AUCUN MARKER DONC A REVOIR
        #pour montrer les sources de radiations (slmt pour testage...a ctually toute cette classe cest slmt pour tester... en vrai elle sert a rien)
        sources = [(4.7071488, 0.0105071), (-6.288366, -0.0121495)]  #coords des sources de raditioaons
        
        for i, (x, y) in enumerate(sources):
            marker = Marker()
            marker.header.frame_id = "map" #jsuis jamais sur si je dois mettre map ou odom à clarifier dans un futur proche
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "radiation_sources"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0  
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0 
            marker.color.r = 1.0 
            marker.color.g = 0.0
            marker.color.b = 0.0

            #self.marker_publisher.publish(marker)

    def publish_radiation_data(self):
        pass
        """ msg = Float32()
        msg.data = 0.0 # à remplacer par la vraie valeur
        self.publisher.publish(msg) """
        #self.get_logger().info(f'Publishing radiation data: {msg.data}')
    
def main(args=None):
    rclpy.init(args=args)
    node = RadiationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

