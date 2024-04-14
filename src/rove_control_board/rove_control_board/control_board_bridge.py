from ctypes.wintypes import RGB
from typing import NamedTuple, NoReturn, Tuple
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.waitable
from rove_control_board import api
import capra_micro_comm_py as comm
from std_msgs.msg import String, Float32, Bool, ColorRGBA
from sensor_msgs.msg import Temperature

import serial
import serial.tools.list_ports

DEV = '/dev/ttyACM0'

# TPV bounds ((horiz min, horiz max), (verti min, verti max))
TPV_BOUNDS:Tuple[Tuple[float, float], Tuple[float,float]] = ((-180, 180), (-45, 90))

@api.setTPVPosition.preCall
def setTPVPositionValidator(pos:api.Vector2D) -> NoReturn:
    if TPV_BOUNDS[0][0] <= pos[0] <= TPV_BOUNDS[0][1] and TPV_BOUNDS[1][0] <= pos[1] <= TPV_BOUNDS[1][1]:
        return
    raise ValueError('Value for TPV out of range')

def findDevice():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        print(p)
        print(p.usb_info())
        print(p.usb_description())

class Bridge(Node):
    def __init__(self):
        super().__init__('control_board_bridge')
        
        
        

def main(args=None):
    print(api.manager.generateAPI())
    api.manager.port = DEV
    api.manager.baud = 9600
    api.manager._stream.rts = True
    api.manager._stream.open()
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