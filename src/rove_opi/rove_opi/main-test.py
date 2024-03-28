# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import cv2
from rclpy.node import Node

from std_msgs.msg import String


class OpiPublisher(Node):

    def __init__(self):
        super().__init__('opi_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        self.i = 0

    def callback(self):
        # Get image from ROS
        img = cv2.imread("src/rove_opi_publisher/rove_opi_publisher/2.png", cv2.IMREAD_COLOR)
        best = None
    
        _, (indices, trapezoids, scores) = finder.find2(img)
        if len(trapezoids) > 0:
            selectedScores = np.array([scores[i] for i in indices])
        
            bestIdx = np.where(selectedScores == selectedScores.max())
            best = trapezoids[bestIdx]
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    opi_publisher = OpiPublisher()

    rclpy.spin(opi_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    opi_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
