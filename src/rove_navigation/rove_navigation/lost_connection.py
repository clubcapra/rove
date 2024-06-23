import socket
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import constants

# Node to handle when the robot loses connection with internet, stores waypoints to fall back to to try to get back connection
class LostNetworkFallback(Node):
    
    def __init__(self):
        self.NB_FALLBACKS=100
        self.DISTANCE_BETWEEN_FALLBACKS = 2 # meters
        self.NB_MAX_FAILED_CONNS = 3
        self.nbFailedConns = 0
        self.fallbackIndex = 0
        self.PING_RATE = 1 # seconds
        
        self.ONLINE = 'ONLINE'
        self.OFFLINE = 'OFFLINE'
        
        self.state = constants.OFFLINE
        self.goal_handle = None
        
        super().__init__('lost_network_fallback')
        self.create_timer(self.PING_RATE, self.check_connection)
        self.odometry_subscription = self.create_subscription(
            Odometry, '/odometry/local', self.odometry_callback, 10)
        self.action_client = ActionClient(self, LostNetworkFallback, 'lost_network_fallback')
        
        #list of potential fallback positions, always will be of size self.NB_FALLBACKS, ascending order
        self.fallback_position = [Point(x=0.0, y=0.0, z=0.0) for _ in range(self.NB_FALLBACKS)]
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
    
    # Check connection to google.com TODO: check connection to UI user instead
    def check_connection(self):
        try:
            socket.setdefaulttimeout(1) # in seconds
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(("8.8.8.8", 53))
            self.nbFailedConns = -1
            if self.goal_handle != None:
                self.action_client._cancel_goal_async(self.goal_handle)
                self.goal_handle = None
        except:
            self.nbFailedConns += 1
            if self.nbFailedConns == self.NB_MAX_FAILED_CONNS:
                # TODO: cancel current state
                self.navigate_to_fallback(self.fallbackIndex)
    
    def odometry_callback(self, msg : Odometry):
        # If robot at self.DISTANCE_BETWEEN_FALLBACKS distance or more from last fallback, add new fallback, delete the last one from the list 
        self.current_position = Point(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, )
        if(np.linalg.norm(self.current_position-self.fallback_position[0]) >= self.DISTANCE_BETWEEN_FALLBACKS):
            self.fallback_position.insert(0, self.current_position)
            self.fallback_position.pop()

    def navigate_to_fallback(self, index):
        assert(index < self.DISTANCE_BETWEEN_FALLBACKS)
        # Prepare the goal message and send it
        goal_pose = PoseStamped()
        goal_pose.header = Header(frame_id='map')
        
        goal_pose.pose.position = self.fallback_position[index]
        goal_pose.pose.orientation.w = 1.0  # Assuming no orientation preference

        self.get_logger().info(f'Navigating to fallback at ({self.fallback_position.__str__()})')
        self.action_client.wait_for_server()
        self.send_goal(goal_pose)

    def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.goal_handle = self.action_client.send_goal_async(goal_msg,self.fallback_point_reached).result()
        self.get_logger().info('Goal sent to navigation system.')
        
    def fallback_point_reached(self):
        if self.state == self.OFFLINE:
            self.fallbackIndex+=1
            self.navigate_to_fallback(self.fallbackIndex)

def main(args=None):
    rclpy.init(args=args)
    node = LostNetworkFallback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
