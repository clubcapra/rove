from __future__ import annotations

from abc import ABC, abstractmethod
import os
import rclpy
from rclpy.node import Node
import sys

from std_msgs.msg import String, Float32, Bool, ColorRGBA
from sensor_msgs.msg import Temperature

# Protos
API_PATH = os.getcwd().split('src')[0] + '/src/rove_control_board'
sys.path.append(API_PATH)

from api.python.Bridge_pb2 import Color, DataFrameRequest, DataFrameResponse, RotationXY, \
    StatusCode, TPVMode, ANGLE, VELOCITY, SUCCESS, IDLE, FAILED

class CanBusManager(ABC):
    def __init__(self):
        pass
    
    @abstractmethod
    def write(self, data:bytes): ...
    
    @abstractmethod
    def read(self) -> bytes: ...

class TopicExplorer:
    def __init__(self, cls:type):
        self.cls = cls
        
    def getTopics(self):
        pass
        

class Bridge(Node):
    def __init__(self):
        super().__init__('control_board_bridge')
        

def main(args=None):
    color = Color()
    print(color.listFields)
    rclpy.init(args=args)

    bridge = Bridge()

    rclpy.spin(bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()