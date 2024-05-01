from __future__ import annotations
from math import isinf, isnan, nan
import math
import os
from time import sleep
from types import TracebackType
from typing import Any, Callable, NamedTuple, NoReturn, Tuple

import can
from cv2 import FAST_FEATURE_DETECTOR_FAST_N

import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.waitable
from rove_control_board import api, canutils
import capra_micro_comm_py as comm
from std_msgs.msg import String, Float32, Bool, ColorRGBA, Byte, Int16, UInt16, Int32, UInt32
from geometry_msgs.msg import PoseStamped, AccelStamped, TwistStamped, Vector3Stamped, Vector3, Twist, Accel, Pose, Quaternion, Point

import serial
import serial.tools.list_ports
import can.interfaces.socketcan

DEV_ID_PREFIX = 'usb-Protofusion_Labs_CANable'
DEV = '/dev/ttyACM0/'

# TPV bounds ((horiz min, horiz max), (verti min, verti max))
TPV_BOUNDS:Tuple[Tuple[float, float], Tuple[float,float]] = ((-180, 180), (-45, 90))
MAX_SPEED_X = 10000
MAX_SPEED_Y = 10000

@api.setServoPosition.preCall
def setServoPositionValidator(pos:api.Vector2D) -> NoReturn:
    if all([lb <= p <= hb for p, (lb, hb) in zip([pos.x, pos.y], TPV_BOUNDS)]):
        return
    raise ValueError(f'Servo position out of range: {pos.x} {pos.y}')

@api.setServoSpeed.preCall
def setServoSpeedValidator(speed:api.Vector2D):
    if not (0 <= abs(speed.x) <= MAX_SPEED_X):
        raise ValueError(f'Servo x speed out of range: {speed.x}')
    
    if not (0 <= abs(speed.y) <= MAX_SPEED_Y):
        raise ValueError(f'Servo y speed out of range: {speed.y}')
    
@api.setServoXAcc.preCall
def setServoXAccValidator(acc:comm.Byte):
    if not (0 <= acc.b < 256):
        raise ValueError(f'Servo x acc out of range {acc.b}')
    
@api.setServoYAcc.preCall
def setServoYAccValidator(acc:comm.Byte):
    if not (0 <= acc.b < 256):
        raise ValueError(f'Servo y acc out of range {acc.b}')

def findDevice():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        print(p)
        print(p.usb_info())
        print(p.usb_description())

def debugFunc(func:Callable[[Bridge], Any]):
    def pred(self:Bridge, *args, **kwargs):
        self.get_logger().debug(f"{func.__name__}({args}, {kwargs})")
        return func(self, *args, **kwargs)
    return pred

class Bridge(Node):
    def __init__(self):
        super().__init__('control_board_bridge', namespace='control_board_bridge')
        self.servo_pos = self.create_publisher(Vector3Stamped, 'servo_pos', 0)
        self.servo_set_pos = self.create_subscription(Vector3Stamped, 'servo_set_pos', self.setPos, 3)
        self.servo_set_vel = self.create_subscription(Vector3Stamped, 'servo_set_vel', self.setVel, 3)
        self.servo_set_acc = self.create_subscription(Vector3Stamped, 'servo_set_acc', self.setAcc, 3)
        self.servo_set_mode = self.create_subscription(UInt32, 'servo_set_mode', self.setMode, 3)
        self.led_front = self.create_publisher(Bool, 'led_front', 0)
        self.led_front_set = self.create_subscription(Bool, 'led_front_set', self.setLEDFront, 3)
        self.led_back = self.create_publisher(Bool, 'led_back', 0)
        self.led_back_set = self.create_subscription(Bool, 'led_back_set', self.setLEDBack, 3)
        self.led_strobe = self.create_publisher(Bool, 'led_strobe', 0)
        self.led_strobe_set = self.create_subscription(Bool, 'led_strobe_set', self.setLEDStrobe, 3)
        self.create_timer(1/30, self.statusReport)
        
        self._led_front = False
        self._led_back = False
        self._led_strobe = False
        self._pos = [0,0]
        self._set_pos = [0,0]
        self._set_vel = [0,0]
        self._set_acc = [0,0]
        self._connected = False
    
    def checkAPI(self):
        from rove_control_board.api import manager
        if (not manager.apiCheck()):
            self.get_logger().fatal("API hash mismatch between rove_control_board and control board microcontroller")
    
    @debugFunc
    def setPos(self, pos:Vector3Stamped):
        try:
            r = api.setServoPosition(api.Vector2D(pos.vector.x, pos.vector.y))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setPos)")
        except ValueError as e:
            self.get_logger().error(e)
    
    @debugFunc
    def setVel(self, vel:Vector3Stamped):
        try:
            r = api.setServoSpeed(api.Vector2D(vel.vector.x, vel.vector.y))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setVel)")
        except ValueError as e:
            self.get_logger().error(e)
            
    @debugFunc
    def setAcc(self, acc:Vector3Stamped):
        try:
            r = api.setServoXAcc(comm.Byte(acc.vector.x))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setAcc X)")
            r = api.setServoYAcc(comm.Byte(acc.vector.y))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setAcc Y)")
        except ValueError as e:
            self.get_logger().error(e)
    
    @debugFunc
    def setMode(self, mode:UInt32):
        raise NotImplementedError()
        
    @debugFunc
    def setLEDFront(self, state:Bool):
        r = api.setLEDFront(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDFront)")
    
    @debugFunc
    def setLEDBack(self, state:Bool):
        r = api.setLEDBack(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDBack)")
    
    @debugFunc
    def setLEDStrobe(self, state:Bool):
        r = api.setLEDStrobe(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDStrobe)")
    
    @debugFunc
    def getPos(self) -> Vector3Stamped:
        r = Vector3Stamped()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = 'getPos'
        r.vector.x = float(self._pos[0])
        r.vector.y = float(self._pos[1])
        return r
            
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
        r = api.getReport(comm.Void())
        if r.errorCode != api.ErrorCode.ERNone.value:
            self.get_logger().error(f"Got {api.ErrorCode(r.errorCode).name} from report.")
        
        self.servo_pos.publish(self.getPos())
        self.led_front.publish(self.getLEDFront())
        self.led_back.publish(self.getLEDBack())
        self.led_strobe.publish(self.getLEDStrobe())
        
def finddir(directory:str, prefix:str):
    directory = directory.removesuffix('/')
    lst = os.listdir(directory)
    print(lst)
    
    res = []
    for f in lst:
        if f.startswith(prefix):
            ff = directory + '/' + f
            fff = '/dev' + os.readlink(ff).removeprefix('../..')
            print(f'{ff} -> {fff}')
            res.append(fff)
    print(res)
    return res
        
def openCan():
    from rove_control_board.api import manager
    
    working = []
    unsupported = ['canalystii', 'cantact', 'nixnet', 'usb2can', 'ixxat', 'etas', 'gs_usb', 'neousys', 'nican', 'systec', 'vector', 'kvaser', 'iscan']
    for itf in can.interfaces.VALID_INTERFACES:
        if itf in unsupported:
            continue
        for chan in finddir('/dev', 'ttyACM'):
            try:
                if manager._stream is not None:
                    manager._stream.stop()
                    manager._stream = None
                manager.interface = itf
                manager.channel = chan
                p = manager.ping()
                print(f"Ping: {round(p, 2)}ms")
                working.append((chan, itf))
                
            except can.CanInterfaceNotImplementedError as e:
                unsupported.append(itf)
                print(f'{itf} not implemented')
                break
            except serial.SerialException as e:
                print(e)
            except ValueError as e:
                if len(e.args) > 0 and e.args[0] == 'channel must be an integer':
                    unsupported.append(itf)
                    break
                print(e)
            except TypeError as e:
                if len(e.args) > 0 and e.args[0] == 'len() takes exactly one argument (2 given)':
                    raise e
                print(e)
            except OSError as e:
                if len(e.args) > 0 and isinstance(e.args[0], str) and 'library not found' in e.args[0]:
                    print(e.args[0])
                    print('please install it')
            except can.CanOperationError as e:
                print(e)
            except can.CanInitializationError as e:
                print(e)
            except ImportError as e:
                print(e)
            except Exception as e:
                raise e
                print(e)
    print(unsupported)
    return working

def openSLCan(dev:str=DEV, prefix:str=None, tryOtherPorts:bool=False):
    from rove_control_board.api import manager
    
    
    if dev is None:
        devs = finddir('/dev/serial/by-id', prefix)
    else:
        devs = [dev]
    
    if tryOtherPorts:
        devs.extend(finddir('/dev', 'ttyACM'))
    
    print(can.interface.detect_available_configs('serial'))

    p = nan
    while True:
        for chan in devs:
            p = nan
            try:
                # if manager._stream is not None:
                #     manager._stream.stop()
                #     manager._stream = None
                manager.interface = 'serial'
                manager.channel = chan
                manager._bus = can.interface.Bus(interface='serial', channel=chan)
                manager._default()
                manager._stream = canutils.CanBusStream(manager._bus, manager._maxSize()*2, manager._notifier)
                p = manager.ping()
                print(f"Ping: {round(p, 2)} ms on {chan}")
                if not isnan(p):
                    break
                
            except can.CanInitializationError as e:
                print(e)
            except can.CanOperationError as e:
                print(e)
            except Exception as e:
                raise e
                print(e)
        if not isnan(p) and not isinf(p):
            break
        else:
            sleep(1)

def testCan():
    # Candlelight firmware on Linux
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=125000)

    # Stock slcan firmware on Linux
    # bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000)

    # Stock slcan firmware on Windows
    # bus = can.interface.Bus(bustype='slcan', channel='COM0', bitrate=500000)

    i = 0
    while True:
        msg = can.Message(arbitration_id=0x446,
                        data=[i, 25, 0, 1, 3, 1, 4, 1],
                        is_extended_id=False)
        i = (i+1) % 255

        try:
            bus.send(msg)
            print("Message sent on {}".format(bus.channel_info))
        except can.CanError:
            print("Message NOT sent")
        sleep(0.5)
    

def openSocket():
    from rove_control_board.api import manager
    
    print(can.interface.detect_available_configs('socketcan'))
    p = nan
    while True:
        for chan in ['can0']:
            p = nan
            try:
                manager.interface = 'socketcan'
                manager.channel = chan
                manager.bitrate = 250000
                p = manager.ping()
                print(f"Ping: {round(p*1000, 2)} us on {chan}")
                if not isnan(p):
                    break
                
            except can.CanInitializationError as e:
                print(e)
            except can.CanOperationError as e:
                print(e)
        if not isnan(p) and not isinf(p):
            break
        else:
            sleep(1)

def main(args=None):
    # testCan()
    from rove_control_board.api import manager
    
    # print(manager.generateAPI())
    
    if isinstance(manager, comm.SerialCommandManager):
        # Serial comm 
        manager.port = DEV
        manager.baud = 9600
        manager._stream.rts = True
        manager._stream.open()
    elif isinstance(manager, canutils.CanBusCommandManager):
        # CAN comm
        # res = openCan()
        # print(res)
        # openSLCan(dev=None, prefix=DEV_ID_PREFIX)
        openSocket()
    
    rclpy.init(args=args)
    
    bridge = Bridge()
    bridge.checkAPI()
    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()