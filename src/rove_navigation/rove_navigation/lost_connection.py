import time
import socket
from threading import Thread

from sympy import Point
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateThroughPoses
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Node to handle when the robot loses connection with internet, stores waypoints to fall back to to try to get back connection
class LostNetworkFallback(Node):
    
    def __init__(self):        
        self.DISTANCE_BETWEEN_FALLBACKS = 0.2 # meters
        self.NB_MAX_FAILED_CONNS = 5
        self.PING_RATE = 1.0 # seconds
        self.PING_TIMEOUT = 2 # seconds
        self.ADDRESS_TO_PING = '8.8.8.8'
        self.PORT_TO_PING = 53
        
        self.ONLINE = 'ONLINE'
        self.OFFLINE = 'OFFLINE'
        
        self.state = self.OFFLINE
        self.nbFailedConns = 0
        self.fallbackIndex = 0
                
        super().__init__('lost_network_fallback')
        self.get_logger().info('Initiating LostNetworkFallback node')
        self.create_timer(self.PING_RATE, self.fallback_behavior)
        self.odometry_subscription = self.create_subscription(
            Odometry, '/odometry/local', self.odometry_callback, 10)
        
        # Thread to execute goal pose posting (because send_goal_async()'s callback was called before reaching actual goal pose)
        self.connection_thread = Thread(target=self.check_connection)
        self.connection_thread.start()
        
        # Navigator object for nav2
        self.navigator = BasicNavigator()
        
        # list of potential fallback positions, ascending order
        self.fallback_path = Path()
        self.fallback_path.header = Header(frame_id='map')
        
        self.current_position = PoseStamped()
        self.current_position.header.frame_id = 'map'
        self.current_position.header.stamp = self.navigator.get_clock().now().to_msg()
        self.current_position.pose.position.x = 0.0
        self.current_position.pose.position.y = 0.0
        self.current_position.pose.position.z = 0.0
        self.current_position.pose.orientation.w = 1.0
        self.fallback_path.poses.append(self.current_position)
    
        self.ini_pos = False
        self.poses_remaining = 0
        

    # Check connection to google.com TODO: check connection to UI user instead
    # Gets called on a different thread
    def check_connection(self):
        while True:
            start = time.time()
            try:
                socket.setdefaulttimeout(self.PING_TIMEOUT)
                socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((self.ADDRESS_TO_PING, self.PORT_TO_PING))
                # if this following code runs, the connection was established
                self.nbFailedConns = 0
                if self.state != self.ONLINE:
                    self.get_logger().info('Connection re-established to server! terminating fallback state')
                self.state = self.ONLINE
            except socket.error as e:
                self.nbFailedConns += 1
                if self.nbFailedConns >= self.NB_MAX_FAILED_CONNS:
                    self.state = self.OFFLINE
                    if self.nbFailedConns == self.NB_MAX_FAILED_CONNS: 
                        self.get_logger().warn(f'Connection Error!!! ping to server failed {self.nbFailedConns} '+
                                           'times in a row, activating fallback state, the robot will try to retrace its steps.')
            delay = time.time() - start
            if(delay < self.PING_RATE): time.sleep(delay)
    
    # gets called on a ros timer (main thread)
    def fallback_behavior(self):
        if self.state==self.OFFLINE and self.poses_remaining > 0:
            self.get_logger().info('nb poses remaingin: '+ str(self.poses_remaining))
            self.navigate_poses()              

        # self.get_logger().warn(f'Navigating to fallback at ({self.fallback_path[self.fallbackIndex].__str__()})')

        # self.fallbackIndex+=1
        # if self.fallbackIndex < len(self.fallback_path.poses):
        #     self.get_logger().warn(f'Fallback {self.fallbackIndex} reached. Will attempt to reach next one if still offline.')
            
        # if self.fallbackIndex == len(self.fallback_path.poses):
        #     self.get_logger().error(f'Last fallback reached, waiting and hoping for connection to come back :(')
    
    def navigate_poses(self):
        self.navigator.goThroughPoses(self.fallback_path.poses)
        i = 0
        while not self.navigator.isTaskComplete():
        
            if self.state == self.ONLINE:
                self.fallbackIndex = 0
                self.navigator.cancelTask()
                self.fallback_path[self.fallbackIndex:]
                return
            
            feedback:NavigateThroughPoses.Feedback = self.navigator.getFeedback()
            self.poses_remaining = feedback.number_of_poses_remaining
            if feedback and i % 5 == 0:
                # navigation timeout
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
            i+=1
            
        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.FAILED:
            self.get_logger().error(f'Couldn\'t proceed through fallbacks, reason unknown')
        else :
            self.get_logger().warn(f'nav2 navigator task returned with unknown TaskResult')
            
    def odometry_callback(self, msg : Odometry):
        self.current_position = PoseStamped()
        self.current_position.pose.position = msg.pose.pose.position
        self.current_position.pose.orientation = msg.pose.pose.orientation
        self.current_position.header.frame_id = 'map'
        self.current_position.header.stamp = self.navigator.get_clock().now().to_msg()
        
        if not self.ini_pos:
            self.navigator.setInitialPose(self.current_position)
            self.ini_pos = True
        # If robot at self.DISTANCE_BETWEEN_FALLBACKS distance or more from last fallback, add new fallback point 
        elif(self.state == self.ONLINE):
            dist = [self.current_position.pose.position.x - self.fallback_path.poses[0].pose.position.x,
                    self.current_position.pose.position.y - self.fallback_path.poses[0].pose.position.y,
                    self.current_position.pose.position.z - self.fallback_path.poses[0].pose.position.z]

            if(np.linalg.norm(dist) >= self.DISTANCE_BETWEEN_FALLBACKS):
                self.fallback_path.poses.insert(0, self.current_position)
                self.get_logger().info(f'New fallback position added: {self.fallback_path.poses[0].__str__()} total fallback points: {len(self.fallback_path.poses)}')            

def main(args=None):
    rclpy.init(args=args)
    node = LostNetworkFallback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
