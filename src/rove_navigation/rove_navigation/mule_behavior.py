import time
import socket
from threading import Thread

from sympy import Point
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateThroughPoses
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rove_navigation.behavior.mule_constants as consts

# Node to handle when the robot loses connection with internet, stores waypoints to fall back to to try to get back connection
class MuleBehavior(Node):
    def __init__(self):        
        super().__init__('mule_behavior')
        self.subscription_person_position = self.create_subscription(
            String,
            '/tracking/state',
            self.state_listener,
            10)
        self.pub_mule_state = self.create_publisher(
            String,
            '/mule_state',
            10)
        self.goal_update_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)
        self.subscription_person_position = self.create_subscription(
            String,
            '/navigate_to_pose/_action/status',
            self.state_listener,
            10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry/local',
            self.odom_listener,
            10)
        
        # nav variables
        self.startPoint: PoseStamped
        self.endpoint: PoseStamped
        self.goal_point: PoseStamped
        self.currentPos: PoseStamped
        self.navigator = BasicNavigator()
        
        
        self.state = ''
        self.change_state(consts.START)
                
        self.get_logger().info('Initiating MuleBehavior node')
        
        # Thread to execute goal pose posting for rewind
        self.connection_thread = Thread(target=self.rewind_behavior)
        self.connection_thread.start()
    
    def change_state(self, state):
        if(state != self.state):
            self.state = state
            self.pub_mule_state.publish(String(data=state))
       
    # Subscribed to /odometry/local
    def odom_listener(self, msg:Odometry):
        self.currentPos = PoseStamped()
        self.currentPos.header.frame_id = msg.header.frame_id
        self.currentPos.header.stamp = msg.header.stamp
        self.currentPos.pose = msg.pose.pose
    
    # Subscribed to /tracking/state, handles state/figure changes
    def state_listener(self, msg:String):
        if self.state != msg.data:
            returnState = msg.data
            match msg.data:
                case consts.FIGURE_TPOSE:
                    if self.state == consts.START:
                        self.startPoint = self.currentPos
                        self.navigator.setInitialPose(self.startPoint)
                        self.get_logger().info(f'Mule behavior engaged, point A noted at coordonates {self.startPoint.pose.position}')
                    returnState = consts.FOLLOWING                     
                        
                case consts.FIGURE_IDLE:
                    if self.state != consts.REWIND and self.state != consts.PROCESSED_REWIND:
                        returnState=self.state
                    
                case consts.FIGURE_UP:
                    if self.state != consts.REWIND and self.state != consts.PROCESSED_REWIND:
                        if self.state == consts.PAUSE: returnState = consts.FOLLOWING
                        else: returnState = consts.PAUSE
                    
                case consts.FIGURE_BUCKET:
                    if self.state != consts.REWIND and self.state != consts.PROCESSED_REWIND:
                        self.endpoint = self.currentPos
                        returnState = consts.REACH_A
                    
                case _:
                    self.get_logger().error(f'Unimplemented pose returned: {msg.data}')     
                           
            self.change_state(returnState)

    # runs on a thread continuously
    def rewind_behavior(self):
        while self.state != consts.END:
            if self.state == consts.REACH_A:
                self.get_logger().info(f'Going to point A coordonates {self.startpoint.pose.position}')
                self.goal_point = self.startPoint
                self.navigate_to_goal_point()
                self.change_state('REWIND')
            if self.state == consts.REACH_B:
                self.get_logger().info(f'Going to point B coordonates {self.endpoint.pose.position}')
                self.goal_point = self.endpoint
                self.navigate_to_goal_point()
                self.change_state('REWIND')
                
                # goal = 'A'
                # nextgoal = 'B'
                # if self.goal_point == self.startPoint:
                #     goal = 'B'
                #     nextgoal = 'A'
                # if result == TaskResult.SUCCEEDED:
                #     self.get_logger().info(f'Goal {goal} reached! Initiating nav towards point {nextgoal}')
                #     self.goal_point = self.startPoint if self.goal_point == self.endpoint else self.endpoint
                # elif result == TaskResult.FAILED:
                #     self.get_logger().error(f'Failed to reach {goal}! Initiating nav towards point {nextgoal}')
                #     self.goal_point = self.startPoint if self.goal_point == self.endpoint else self.endpoint
                # elif result == TaskResult.CANCELED:
                #     self.get_logger().error(f'Rewinding to point {goal} was cancelled! Initiating nav towards point {nextgoal}')
                # else :
                #     self.get_logger().warn(f'nav2 navigator task returned with unknown TaskResult')
    
    def navigate_to_goal_point(self):
        # Log the navigation target for debugging
        self.get_logger().info(f'Navigating to goal : {self.goal_point}')
        # Publish the goal
        self.goal_update_pub.publish(self.goal_point)
        
    # # Tells navigator to go to current goal Point and processes feedback
    # def navigate_to_goal_point(self):
    #     i = 0
    #     self.navigator.goToPose(self.goal_point)
    #     while not self.navigator.isTaskComplete():
    #         if self.state != consts.PROCESSED_REWIND:
    #             self.navigator.cancelTask()

    #         feedback:NavigateThroughPoses.Feedback = self.navigator.getFeedback()
    #         self.poses_remaining = feedback.number_of_poses_remaining
    #         if feedback and i % 5 == 0:
    #             # navigation timeout
    #             if Duration.from_msg(feedback.navigation_time) > 1.5 * Duration(seconds=feedback.estimated_time_remaining):
    #                 self.navigator.cancelTask()
    #         i+=1

def main(args=None):
    rclpy.init(args=args)
    node = MuleBehavior()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
