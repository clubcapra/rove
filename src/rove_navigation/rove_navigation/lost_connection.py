from asyncio import Future
import socket
from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

# Node to handle when the robot loses connection with internet, stores waypoints to fall back to to try to get back connection
class LostNetworkFallback(Node):
    
    def __init__(self):        
        self.DISTANCE_BETWEEN_FALLBACKS = 0.4 # meters
        self.NB_MAX_FAILED_CONNS = 3
        self.PING_RATE = 1 # seconds
        
        self.ONLINE = 'ONLINE'
        self.OFFLINE = 'OFFLINE'
        
        self.state = self.ONLINE
        self.traveling_to_fallback = False
        self.nbFailedConns = 0
        self.fallbackIndex = 0
                
        super().__init__('lost_network_fallback')
        self.get_logger().info('Initiating LostNetworkFallback node')
        self.create_timer(self.PING_RATE, self.fallback_behavior)
        self.odometry_subscription = self.create_subscription(
            Odometry, '/odometry/local', self.odometry_callback, 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        #list of potential fallback positions, ascending order
        self.fallback_positions = [Point(x=0.0,y=0.0,z=0.0)]
        self.current_position = Point(x=0.0,y=0.0,z=0.0)
        
        self.fallback_positions.pop()
        self.current_position: Point
        # Thread to execute goal pose posting (because send_goal_async()'s callback was called before reaching actual goal pose)
        self.connection_thread = Thread(target=self.check_connection)
        self.connection_thread.start()
    
    # gets called on a ros timer (main thread)
    async def fallback_behavior(self):
        if self.state==self.OFFLINE and self.fallbackIndex < len(self.fallback_positions):
            await self.navigate_to_fallback()
    
    # Check connection to google.com TODO: check connection to UI user instead
    # Gets called on a different thread
    def check_connection(self):
        while True:
            try:
                socket.setdefaulttimeout(1) # in seconds
                socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(("8.8.8.8", 53))
                # if this following code runs, the connection was established
                self.nbFailedConns = 0
                if self.state != self.ONLINE:
                    self.get_logger().info('Connection re-established to server! terminating fallback state')
                    self.fallback_positions[self.fallbackIndex:]
                    self.fallbackIndex = 0
                self.state = self.ONLINE
            except socket.error:
                self.nbFailedConns += 1
                if self.nbFailedConns >= self.NB_MAX_FAILED_CONNS:
                    self.state = self.OFFLINE
                    if self.nbFailedConns == self.NB_MAX_FAILED_CONNS: 
                        self.get_logger().warn(f'Connection Error!!! ping to server failed {self.nbFailedConns} '+
                                           'times in a row, activating fallback state, the robot will try to retrace its steps.')                    

    async def navigate_to_fallback(self):
        # Prepare the goal message and send it
        goal_pose = PoseStamped()
        goal_pose.header = Header(frame_id='map')
        
        goal_pose.pose.position = self.fallback_positions[self.fallbackIndex]
        goal_pose.pose.orientation.w = 1.0  # Assuming no orientation preference

        self.get_logger().warn(f'Navigating to fallback at ({self.fallback_positions[self.fallbackIndex].__str__()})')
        self.action_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_handle = await self.action_client.send_goal(goal_msg) ## the future returned from this does NOT WORK >:(
        while not self.is_robot_close_to_goal():
            if self.state == self.ONLINE: 
                # TODO: cancel current state of general state_handler, right now only cancels return to fallback
                self.action_client._cancel_goal_async(goal_handle) 
                return # end fallback loop
            
        self.fallbackIndex+=1
        if self.fallbackIndex < len(self.fallback_positions):
            self.get_logger().warn(f'Fallback {self.fallbackIndex} reached. Will attempt to reach next one if still offline.')
            
        if self.fallbackIndex == len(self.fallback_positions):
            self.get_logger().error(f'Last fallback reached, waiting and hoping for connection to come back :(')
        
        self.traveling_to_fallback = False
            
        
    def is_robot_close_to_goal(self):
        fall = self.fallback_positions[self.fallbackIndex]
        dist = [self.current_position.x-fall.x,
                self.current_position.y-fall.y,
                self.current_position.z-fall.z]
        self.get_logger().info('fallback_positions[i]: '+str(fall))
        self.get_logger().info('self.current_position: '+str(self.current_position))
        self.get_logger().info('distance: '+str(np.linalg.norm(dist)))
        return np.linalg.norm(dist) <= 0.3
        
    def odometry_callback(self, msg : Odometry):
        self.current_position = Point(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y, z=msg.pose.pose.position.z)
        # Sets the first position
        if len(self.fallback_positions) == 0: 
            self.fallback_positions = [self.current_position] 
            self.get_logger().info(f'First fallback position initiated: {self.fallback_positions[0]}')            
            
        # If robot at self.DISTANCE_BETWEEN_FALLBACKS distance or more from last fallback, add new fallback point 
        if(self.state == self.ONLINE):
            if len(self.fallback_positions) == 0: 
                self.fallback_positions.append(self.current_position)
            else:
                dist = [self.current_position.x - self.fallback_positions[0].x,
                        self.current_position.y - self.fallback_positions[0].y,
                        self.current_position.z - self.fallback_positions[0].z]

                if(np.linalg.norm(dist) >= self.DISTANCE_BETWEEN_FALLBACKS):
                    self.fallback_positions.insert(0, self.current_position)
                    self.get_logger().info(f'New fallback position added: {self.fallback_positions[0].__str__()} total fallback points: {len(self.fallback_positions)}')            

def main(args=None):
    rclpy.init(args=args)
    node = LostNetworkFallback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
