from __future__ import annotations
import functools
from math import isinf, isnan, nan
import math
from time import sleep
import time
from typing import Tuple, TypeVar

import can
import sys
sys.path.append(__file__.removesuffix(f"/{__file__.split('/')[-1]}"))

import rclpy
import rclpy.impl
import rclpy.impl.rcutils_logger
from rclpy.node import Node
from . import api, canutils
import capra_micro_comm_py as comm
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Joy, JointState


def debugFunc(func):
    
    def pred(self:Bridge, *args, **kwargs):
        res = func(self, *args, **kwargs)
        self.get_logger().info(f"{func.__name__}({args}, {kwargs}) = {res}")
        return res
    return functools.update_wrapper(func, pred)

NumberT = TypeVar('NumberT', float, int)
def clamp(value:NumberT, minValue:NumberT, maxValue:NumberT) -> NumberT:
    return min(maxValue, max(minValue, value))

class Bridge(Node):
    def __init__(self):
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        super().__init__('control_board_bridge')
        from api import manager
        
        self.manager = manager
        
        # Setup parameters
        self.channel = self.declare_parameter(
            'channel', 'can0', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Can channel'))
        self.bitrate = self.declare_parameter(
            'bitrate', 500000, ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Can bitrate'))
        self.mock = self.declare_parameter(
            'mock', False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='Mock communication'))
        
        # Setup pubs and subs
        self.led_front = self.create_publisher(Bool, 'led_front', 1)
        self.led_front_set = self.create_subscription(Bool, 'led_front_set', self.setLEDFront, 3)
        self.led_back = self.create_publisher(Bool, 'led_back', 1)
        self.led_back_set = self.create_subscription(Bool, 'led_back_set', self.setLEDBack, 3)
        self.led_strobe = self.create_publisher(Bool, 'led_strobe', 1)
        self.led_strobe_set = self.create_subscription(Bool, 'led_strobe_set', self.setLEDStrobe, 3)

        # Setup api memory
        self.reset()
    
    def reset(self):
        self._led_front = False
        self._led_back = False
        self._led_strobe = False
        self._connected = False
    
    def __enter__(self) -> Bridge:
        self.manager.__enter__()
        return self

    def __exit__(self, *args, **kwargs):
        self.manager.__exit__(*args, **kwargs)
    
    def connect(self):
        if self.mock:
            self._connected = True
            return
        
        if self._connected:
            return
        while rclpy.ok():
            if self.ping() == math.nan:
                self.get_logger().error("Connection")
                sleep(5)
                continue
            if not self.checkAPI():
                sleep(5)
                continue
            self.get_logger().info("API hash matches")
            self._connected = True
            break
    
    def checkAPI(self):
        if self.mock.value:
            return True
        if (not self.manager.apiCheck()):
            self.get_logger().fatal("API hash mismatch between rove_control_board and control board microcontroller")
            return False
        return True
    
    @property
    def nowSeconds(self):
        return time.time()
        # return self.get_clock().now().nanoseconds / 1000000000.0
    
    @debugFunc
    def setLEDFront(self, state:Bool):
        if self.mock.value:
            return
        r = api.setLEDFront(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDFront)")
    
    @debugFunc
    def setLEDBack(self, state:Bool):
        if self.mock.value:
            return
        r = api.setLEDBack(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDBack)")
    
    @debugFunc
    def setLEDStrobe(self, state:Bool):
        if self.mock.value:
            return
        r = api.setLEDStrobe(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDStrobe)")

    @debugFunc
    def setLEDColor(self, id:int, color:Tuple[int, int, int]):
        if self.mock:
            return
        r = api.setRGBLed(api.RGBLed(api.RGB(*color), id))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDColor)")
    
    @debugFunc
    def getLEDFront(self) -> Bool:
        r = Bool()
        r.data = self._led_front
        return r
    
    @debugFunc
    def getLEDBack(self) -> Bool:
        r = Bool()
        r.data = self._led_back
        return r
    
    @debugFunc
    def getLEDStrobe(self) -> Bool:
        r = Bool()
        r.data = self._led_strobe
        return r
    
    @debugFunc
    def statusReport(self):
        self.led_front.publish(self.getLEDFront())
        self.led_back.publish(self.getLEDBack())
        self.led_strobe.publish(self.getLEDStrobe())
        
    @debugFunc
    def ping(self):
        p = self.manager.ping()
        self.get_logger().info(f"Ping: {round(p*1000, 2)} ms")
        
def openSocket(bitrate:int, logger:rclpy.impl.rcutils_logger.RcutilsLogger):
    from api import manager
    
    logger.info(str(can.interface.detect_available_configs('socketcan')))
    p = nan
    while True:
        for chan in ['can0']:
            p = nan
            try:
                manager.interface = 'socketcan'
                manager.channel = chan
                manager.bitrate = bitrate
                manager.remoteID = 0x103
                manager.localID = 0x446
                manager.timeout = 2
                p = manager.ping()
                
                logger.info(f"Ping: {round(p*1000, 2)} ms on {chan}")
                if not isnan(p):
                    break
                
            except can.CanInitializationError as e:
                logger.error(str(e))
            except can.CanOperationError as e:
                logger.error(str(e))
            except OSError as e:
                logger.fatal(f"Socket does not exist:\n{str(e)}")
                exit()
        if not isnan(p) and not isinf(p):
            break
        else:
            sleep(1)

def main(args=None):
    # testCan()
    from api import manager
    
    # Init ros and bridge
    rclpy.init(args=args)
    bridge = Bridge()
    logger = bridge.get_logger()
        
    if isinstance(manager, canutils.CanBusCommandManager):
        # CAN comm
        if not bridge.mock.value:
            openSocket(bridge.bitrate, logger)
    
    try:
        bridge.connect()
        rclpy.spin(bridge)
        rclpy.shutdown()
    except KeyboardInterrupt:
        rclpy.try_shutdown()

if __name__ == '__main__':
    
    main()