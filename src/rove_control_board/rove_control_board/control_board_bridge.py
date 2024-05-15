from __future__ import annotations
from ctypes.wintypes import RGB
from enum import Enum
import enum
import functools
from math import isinf, isnan, nan
import math
from multiprocessing import queues
import os
from sqlite3 import connect
import threading
from time import sleep
import time
from typing import Any, Callable, Dict, NoReturn, ParamSpecArgs, ParamSpecKwargs, Tuple, TypeVar, overload

import can

import rclpy
import rclpy.logging
import rclpy.parameter
from rclpy.node import Node
import rclpy.time
import rclpy.waitable
from rove_control_board import api, canutils
import capra_micro_comm_py as comm
from std_msgs.msg import String, Float32, Bool, ColorRGBA, Byte, Int16, UInt16, Int32, UInt32, Float64MultiArray
from geometry_msgs.msg import PoseStamped, AccelStamped, TwistStamped, Vector3Stamped, Vector3, Twist, Accel, Pose, Quaternion, Point
from sensor_msgs.msg import Joy

import serial
import serial.tools.list_ports
import can.interfaces.socketcan

DEV_ID_PREFIX = 'usb-Protofusion_Labs_CANable'
DEV = '/dev/ttyACM0/'

# TPV bounds ((horiz min, horiz max), (verti min, verti max))
TPV_BOUNDS:Tuple[Tuple[float, float], Tuple[float,float]] = ((-10000, 10000), (-10000, 10000))
MAX_SPEED_X = 10000
MAX_SPEED_Y = 10000

CANBUS_BITRATE = 500000

STEP_COUNT = 4096

def degToStep(deg:float) -> int: 
    pass

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

def debugFunc(func):
    
    def pred(self:Bridge, *args, **kwargs):
        res = func(self, *args, **kwargs)
        self.get_logger().info(f"{func.__name__}({args}, {kwargs}) = {res}")
        return res
    return functools.update_wrapper(func, pred)

NumberT = TypeVar('NumberT', float, int)
def clamp(value:NumberT, minValue:NumberT, maxValue:NumberT) -> NumberT:
    return min(maxValue, max(minValue, value))

# def toDeg(valueStep:int, stepCount:int, offsetDeg:float, minDeg:float, maxDeg:float) -> float:
def toDeg(valueStep:int, stepCount:int) -> float:
    if not isinstance(valueStep, int):
        valueStep = int(valueStep)
    stepDeg = 360.0/stepCount
    absoluteDeg = valueStep*stepDeg
    # relativeDeg = absoluteDeg - offsetDeg
    relativeDeg = absoluteDeg
    # res = clamp(relativeDeg, minDeg, maxDeg)
    res = relativeDeg
    if not isinstance(res, float):
        res = float(res)
    return res

# def toStep(valueDeg:float, stepCount:int, offsetDeg:float, minDeg:float, maxDeg:float) -> int:
def toStep(valueDeg:float, stepCount:int) -> int:
    if not isinstance(valueDeg, float):
        valueDeg = float(valueDeg)
    stepDeg = 360.0/stepCount
    # valueDeg = clamp(valueDeg, minDeg, maxDeg) + offsetDeg
    valueDeg = valueDeg
    absoluteStep = valueDeg/stepDeg
    if not isinstance(absoluteStep, int):
        absoluteStep = int(round(absoluteStep))
    return absoluteStep

Tp = TypeVar('Tp', bound=comm.BinaryData)


class ThreadLock:
    def __init__(self):
        self._innerLock = threading.Lock()
        self._lockDict:Dict[int, threading.Lock] = {}
        
    def acquire(self):
        id = threading.current_thread().native_id
        print(f'>>>{id}')
        with self._innerLock:
            if id not in self._lockDict:
                self._lockDict[id] = threading.Lock()
        self._lockDict[id].acquire()
    
    def release(self):
        id = threading.current_thread().native_id
        with self._innerLock:
            if id not in self._lockDict:
                self._lockDict[id] = threading.Lock()
        self._lockDict[id].release()
        print(f'<<<{id}')
        
    def __enter__(self):
        self.acquire()
        
    def __exit__(self, _, __, ___):
        self.release()
        
_canLock = threading.Lock()

def safe_call(lock:threading.Lock, func:Callable, *args, **kwargs):
    with lock:
        return func(*args, **kwargs)

def callThreadsafe(func:Callable):
    def pred(*args, **kwargs):
        with _canLock:
            return func(*args, **kwargs)
    return functools.update_wrapper(pred, func)

class Bridge(Node):
    def __init__(self):
        super().__init__('control_board_bridge', namespace='control_board_bridge')
        from rove_control_board.api import manager
        self.manager = manager
        
        # Setup parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tpv_x_step_count', rclpy.Parameter.Type.INTEGER),
                ('tpv_x_min', rclpy.Parameter.Type.DOUBLE),
                ('tpv_x_max', rclpy.Parameter.Type.DOUBLE),
                ('tpv_x_offset', rclpy.Parameter.Type.DOUBLE),
                ('tpv_x_speed', rclpy.Parameter.Type.INTEGER),
                ('tpv_y_step_count', rclpy.Parameter.Type.INTEGER),
                ('tpv_y_min', rclpy.Parameter.Type.DOUBLE),
                ('tpv_y_max', rclpy.Parameter.Type.DOUBLE),
                ('tpv_y_offset', rclpy.Parameter.Type.DOUBLE),
                ('tpv_y_speed', rclpy.Parameter.Type.INTEGER),
            ]
        )
        
        # Get parameters
        self._xStepCount = self.get_parameter_or('tpv_x_step_count', STEP_COUNT)
        self._xMin = self.get_parameter_or('tpv_x_min', -180)
        self._xMax = self.get_parameter_or('tpv_x_max', 180)
        self._xOffset = self.get_parameter_or('tpv_x_offset', 180)
        self._xSpeed = self.get_parameter_or('tpv_x_speed', 5000)
        
        self._yStepCount = self.get_parameter_or('tpv_y_step_count', STEP_COUNT)
        self._yMin = self.get_parameter_or('tpv_y_min', -45)
        self._yMax = self.get_parameter_or('tpv_y_max', 90)
        self._yOffset = self.get_parameter_or('tpv_y_offset', 45)
        self._ySpeed = self.get_parameter_or('tpv_y_speed', 3500)
        
        
        # Setup pubs and subs
        self.servo_pos = self.create_publisher(Vector3Stamped, 'servo_pos', 0)
        self.servo_set_pos = self.create_subscription(Vector3Stamped, 'servo_set_pos', callThreadsafe(self.setPos), 3)
        self.servo_set_vel = self.create_subscription(Vector3Stamped, 'servo_set_vel', callThreadsafe(self.setVel), 3)
        self.servo_set_acc = self.create_subscription(Vector3Stamped, 'servo_set_acc', callThreadsafe(self.setAcc), 3)
        # self.servo_set_mode = self.create_subscription(UInt32, 'servo_set_mode', self.setMode, 3)
        self.servo_joy = self.create_subscription(Joy, '/joy', self.joy, 0)
        self.led_front = self.create_publisher(Bool, 'led_front', 0)
        self.led_front_set = self.create_subscription(Bool, 'led_front_set', callThreadsafe(self.setLEDFront), 3)
        self.led_back = self.create_publisher(Bool, 'led_back', 0)
        self.led_back_set = self.create_subscription(Bool, 'led_back_set', callThreadsafe(self.setLEDBack), 3)
        self.led_strobe = self.create_publisher(Bool, 'led_strobe', 0)
        self.led_strobe_set = self.create_subscription(Bool, 'led_strobe_set', callThreadsafe(self.setLEDStrobe), 3)
        
        self._control_mode = api.ServoControlMode.SCMPosition
        self._control_mode_timeout = 5
        self._control_mode_return = 0
        self._last_send = 0
        
        # Setup status report
        self.heartbeat = self.create_timer(1/15, callThreadsafe(self.statusReport))
        # self.create_timer(1/100, self.ping)
        
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Setup api memory
        self.reset()
        
    
    def reset(self):
        self._led_front = False
        self._led_back = False
        self._led_strobe = False
        self._pos = [0,0]
        self._set_pos = [0,0]
        self._set_vel = [5000,5000]
        self._move_vel = [0,0]
        self._move_act_vel = [0,0]
        self._set_acc = [150,150]
        self._connected = False
        self._lastError = api.ErrorCode.ERNone
        self._confIterator = None
        self._configured = False
    
    
    
    def connect(self):
        if self._connected:
            return
        while rclpy.ok():
            try:
                if self.ping() == math.nan:
                    self.get_logger().error("Connection")
                    sleep(5)
                    continue
                if not self.checkAPI():
                    sleep(5)
                    continue
                self.get_logger().info("API hash matches")
                self._connected = True
                self.get_logger().info("Connected")
                break
            except TimeoutError as e:
                self.get_logger().error(f"Timed out: {str(e)}")
                continue
    
    def disconnected(self):
        self.get_logger().error("Connection lost")
        self._connected = False
        # self._configured = False
    
    def deadzone(self, v:float):
        return 0 if abs(v) < 0.075 else v
    
    def checkAPI(self):
        from rove_control_board.api import manager
        if not manager.apiCheck():
            self.get_logger().fatal("API hash mismatch between rove_control_board and control board microcontroller")
            return False
        return True
    
    def _xToDeg(self, step:int) -> float:
        # return toDeg(step, self._xStepCount, self._xOffset, self._xMin, self._xMax)
        return toDeg(step, self._xStepCount)
    
    def _xToStep(self, deg:float) -> int:
        # return toStep(deg, self._xStepCount, self._xOffset, self._xMin, self._xMax)
        return toStep(deg, self._xStepCount)
    
    def _yToDeg(self, step:int) -> float:
        # return toDeg(step, self._yStepCount, self._yOffset, self._yMin, self._yMax)
        return toDeg(step, self._yStepCount)
    
    def _yToStep(self, deg:float) -> int:
        # return toStep(deg, self._yStepCount, self._yOffset, self._yMin, self._yMax)
        return toStep(deg, self._yStepCount)
    
    @overload
    def printErrors(self, error:int): ... @overload
    def printErrors(self, error:api.ErrorCode):
        if not isinstance(error, api.ErrorCode):
            error = api.ErrorCode(error)
            
        
        if error != api.ErrorCode.ERNone and error != self._lastError:
            new = ~self._lastError & error
            if new != api.ErrorCode.ERNone:
                self.get_logger().error(f"Errors: {str(error)} | New: {str(new)}")
                
            solved = ~error & self._lastError
            if solved != api.ErrorCode.ERNone:
                self.get_logger().info(f"Resolved: {str(solved)}")
            
            self._lastError = error
            
    
    @property
    def nowSeconds(self):
        return time.time()
        # return self.get_clock().now().nanoseconds / 1000000000.0
    def _pushbackControlTimeout(self):
        self._control_mode_return = self.nowSeconds + self._control_mode_timeout
        if self._control_mode == api.ServoControlMode.SCMPosition:
            self.setMode(api.ServoControlMode.SCMSpeed)
        r = api.setServoSpeed(api.Vector2D(int(self._move_act_vel[0] * self._xSpeed), int(self._move_act_vel[1] * self._ySpeed)))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from _pushbackControlTimeout)")
    def _checkControlTimeout(self):
        if self._control_mode_return < self.nowSeconds:
            if self._control_mode == api.ServoControlMode.SCMSpeed:
                self.setMode(api.ServoControlMode.SCMPosition)
                r = api.setServoSpeed(api.Vector2D(self._set_vel[0], self._set_vel[1]))
                if not r.b:
                    self.get_logger().warning(f"Last command didn't ack (from _checkControlTimeout)")
            return True
        return False
    
    @debugFunc
    def joy(self, value:Joy):
        x = value.axes[3]
        y = value.axes[4]
        self._move_vel = [self.deadzone(x), self.deadzone(y)]
        if value.buttons[0] == 1:
            self.setLEDColor(0, (255, 0, 0))
        if value.buttons[1] == 1:
            self.setLEDColor(0, (0, 255, 0))
        if value.buttons[2] == 1:
            self.setLEDColor(0, (0, 0, 255))
        if value.buttons[3] == 1:
            self.setLEDColor(0, (0, 0, 0))
        # vel = Vector3Stamped()
        # vel.header.stamp = self.get_clock().now().to_msg()
        # vel.header.frame_id = 'servo_move_vel'
        # vel.vector.x = x
        # vel.vector.y = y
        # self.setMoveVel(vel)
    
    @debugFunc
    def setPos(self, pos:Vector3Stamped):
        if not self._checkControlTimeout():
            return
        try:
            r = api.setServoPosition(api.Vector2D(self._xToStep(pos.vector.x - self._xMin), self._yToStep(pos.vector.y - self._yMin)))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setPos)")
        except ValueError as e:
            self.get_logger().error(str(e))
    
    @debugFunc
    def setVel(self, vel:Vector3Stamped):
        if not self._checkControlTimeout():
            return
        try:
            r = api.setServoSpeed(api.Vector2D(int(vel.vector.x), int(vel.vector.y)))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setVel)")
        except ValueError as e:
            self.get_logger().error(str(e))
        
    @debugFunc
    def setMoveVel(self, vel:Vector3Stamped):
        try:
            x = vel.vector.x
            y = vel.vector.y
            self._move_vel = [self.deadzone(x), self.deadzone(y)]
        except ValueError as e:
            self.get_logger().error(str(e))
            
    @debugFunc
    def setAcc(self, acc:Vector3Stamped):
        try:
            r = api.setServoXAcc(comm.Byte(int(acc.vector.x)))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setAcc X)")
            r = api.setServoYAcc(comm.Byte(int(acc.vector.y)))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setAcc Y)")
        except ValueError as e:
            self.get_logger().error(str(e))
    
    @debugFunc
    def setMode(self, mode:api.ServoControlMode):
        r = api.setServoControlMode(comm.UShort(mode.value))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setMode)")
        else:
            self._control_mode = mode
          
    @debugFunc
    def getMode(self):
        r = api.getServoControlMode()
        try:
            return api.ServoControlMode(r.s)
        except ValueError as e:
            self.get_logger().error(str(e))
        
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
    def setLEDColor(self, id:int, color:Tuple[int, int, int]):
        r = api.setRGBLed(api.RGBLed(api.RGB(*color), id))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDColor)")
    
    @debugFunc
    def getPos(self) -> Vector3Stamped:
        r = Vector3Stamped()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = 'getPos'
        r.vector.x = self._xToDeg(self._pos[0] + self._xMin)
        r.vector.y = self._yToDeg(self._pos[1] + self._yMin)
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
            try:
                self.printErrors(r.errorCode)
            except ValueError as e:
                self.get_logger().error(str(e))
        if r.statusCode == api.StatusCode.STNotInitialized.value:
            self.get_logger().warning("Not initialized")
            # sleep(2)
            return
        if r.statusCode == api.StatusCode.STInitialized.value and not self._configured:
            if self._confIterator is None:
                self.get_logger().info("Configuring")
                self._confIterator = iter(self.sendConfig())
                # sleep(2)
            next(self._confIterator)
            if self._configured:
                self.get_logger().info("Done")
            return
            
        self._pos[0] = r.pos.x
        self._pos[1] = r.pos.y
            
        if self._move_act_vel[0] != self._move_vel[0] or self._move_act_vel[1] != self._move_vel[1]:
            self._last_send = self.nowSeconds
            self._move_act_vel[0] = self.deadzone(self._move_vel[0])
            self._move_act_vel[1] = self.deadzone(self._move_vel[1])
            self._pushbackControlTimeout()
            # r = api.setServoSpeed(api.Vector2D(int(self._move_act_vel[0] * self._xSpeed), int(self._move_act_vel[1] * self._ySpeed)))
            # if not r.b:
            #     self.get_logger().warning(f"Last command didn't ack (from setMoveVel)")
        
        
        self.servo_pos.publish(self.getPos())
        self.led_front.publish(self.getLEDFront())
        self.led_back.publish(self.getLEDBack())
        self.led_strobe.publish(self.getLEDStrobe())
        
        
    @debugFunc
    def sendConfig(self):
        delay = 0.5
        def ensure(call:Callable[[Tp], comm.Bool_], arg:Tp):
            self.get_logger().info(f"Configuring: {call.__name__}")
            while True:
                try:
                    r = call(arg)
                    if not r:
                        self.get_logger().warning(f"Last command didn't ack {call.__name__} (from sendConfig)")
                        yield
                        continue
                    break
                except TimeoutError as e:
                    self.get_logger().error(f"Timed out: {str(e)} (from {call.__name__})")
                    yield
                # sleep(delay)
            self.get_logger().info(f"Configured: {call.__name__}")
            # sleep(delay)
        
        res = [
            # ensure(
            #     api.configure, 
            #     api.Configuration(
            #         api.Bounds(int(self._xToStep(0)), int(self._xToStep(self._xMax - self._xMin))),
            #         api.Bounds(int(self._xToStep(0)), int(self._xToStep(self._yMax - self._yMin))))),
            ensure(api.setServoControlMode, comm.UShort(api.ServoControlMode.SCMPosition.value)),
            ensure(api.setServoXAcc, comm.Byte(int(self._set_acc[0]))),
            ensure(api.setServoYAcc, comm.Byte(int(self._set_acc[1]))),
            ensure(api.setServoSpeed, api.Vector2D(int(self._set_vel[0]), int(self._set_vel[1]))),
            ensure(api.setServoPosition, api.Vector2D(int(self._set_pos[0]), int(self._set_pos[1]))),
        ]
        
        for r in res:
            yield from r
        self._configured = True
        yield
        
    @debugFunc
    def ping(self):
        from rove_control_board.api import manager
        p = manager.ping()
        self.get_logger().info(f"Ping: {round(p*1000, 2)} ms")
        
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
                print(f"Ping: {round(p*1000, 2)} ms on {chan}")
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
                manager.bitrate = CANBUS_BITRATE
                manager.remoteID = 0x103
                manager.localID = 0x446
                manager.timeout = 2
                p = manager.ping()
                # pp = randint(-1000, 1000)
                # print(f"Sending {pp}")
                # manager._pingcmd(comm.Int(pp))
                
                
                
                print(f"Ping: {round(p*1000, 2)} ms on {chan}")
                if not isnan(p):
                    break
                
            except can.CanInitializationError as e:
                print(e)
            except can.CanOperationError as e:
                print(e)
            except TimeoutError as e:
                print(f"Timed out: {str(e)}")
        if not isnan(p) and not isinf(p):
            break
        else:
            sleep(1)

def simpleSocketOpen():
    from rove_control_board.api import manager
    
    print(can.interface.detect_available_configs('socketcan'))
    chan = 'can0'
    while True:
        try:
            manager.interface = 'socketcan'
            manager.channel = chan
            manager.bitrate = CANBUS_BITRATE
            manager.remoteID = 0x103
            manager.localID = 0x446
            manager.timeout = 2
        except can.CanInitializationError as e:
            print(e)
        except can.CanOperationError as e:
            print(e)

def main(args=None):
    # testCan()
    from rove_control_board.api import manager
    
    # print(manager.generateAPI())
    socketOpenner:Callable[[],None] = None
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
    try:
        bridge = Bridge()
        while True:
            try:
                bridge.connect()
                rclpy.spin(bridge)
            except TimeoutError as e:
                bridge.disconnected()
                bridge.get_logger().error(f"Timed out: {str(e)}")
    except KeyboardInterrupt:
        rclpy.shutdown()
        pass

if __name__ == '__main__':
    
    main()