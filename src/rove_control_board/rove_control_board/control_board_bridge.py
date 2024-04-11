import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.waitable
from rove_control_board import api, capra_micro_comm
from std_msgs.msg import String, Float32, Bool, ColorRGBA
from sensor_msgs.msg import Temperature

import serial
import serial.tools.list_ports
import struct

DEV = '/dev/ttyACM1'


def findDevice():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        print(p)
        print(p.usb_info())
        print(p.usb_description())

class Bridge(Node):
    def __init__(self):
        super().__init__('control_board_bridge')
        self.offset = 2
        self.timer = self.create_timer(1, self.timerCB)
        self.state = 1
        self.value = 0
        
        
    def timerCB(self):
        if self.offset > 0 :
            self.offset -= 1
            return
        # if self.state:
        #     s = api.Status(api.StatusCode.ERROR)
        #     print(s.statusCode)
        #     print(api.patate(s))
        # #     api.ledOn()
        # else:
        #     s = api.Status(api.StatusCode.IDLE)
        #     print(s.statusCode)
        #     print(api.patate(s))
        #     api.ledOff()
        # api.setLedState(api.State(self.state))
        api.loopback(api.UInt8(self.value))
        self.state = (self.state + 1) %2
        self.value = (self.value + 1) % 10
        
        

def main(args=None):
    # print(api.manager.buildAPI())
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