from __future__ import annotations
import functools
from math import isinf, isnan, nan
import math
import os
from time import sleep
import time
from typing import NoReturn, Tuple, TypeVar

import can

import rclpy
import rclpy.client
import rclpy.impl
import rclpy.impl.rcutils_logger
import rclpy.logging
import rclpy.parameter
from rclpy.node import Node
import rclpy.time
import rclpy.waitable
from rove_control_board import api, canutils
from rove_control_board.canutils import CANBUS_BITRATE
import capra_micro_comm_py as comm
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Joy, JointState

import serial
import serial.tools.list_ports
import can.interfaces.socketcan

DEV_ID_PREFIX = 'usb-Protofusion_Labs_CANable'
DEV = '/dev/ttyACM0/'

# TPV bounds ((horiz min, horiz max), (verti min, verti max))
TPV_BOUNDS:Tuple[Tuple[float, float], Tuple[float,float]] = ((-10000, 10000), (-10000, 10000))
MAX_SPEED_X = 10000
MAX_SPEED_Y = 10000


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

def deg2rad(deg:float) -> float:
    return deg * (3.14159265359 / 180.0)

def rad2deg(rad:float) -> float:
    return rad / (3.14159265359 / 180.0)

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

class ServoMock:
    def __init__(self, velocity:float):
        self.set_pos = 0
        self.set_vel = velocity
        self.set_move_vel = 0
        self.set_acc = 500
        self.actual_position = 0
        self.last_update = time.time()
        
    def deltaTime(self):
        return time.time() - self.last_update
    
    def move_vel(self, vel:float):
        self.update()
        self.set_move_vel = vel
    
    def move_pos(self, pos:float):
        self.update()
        self.set_pos = pos
        self.set_move_vel = None
    
    def update(self):
        if self.set_move_vel is None:
            movement = self.deltaTime() * self.set_vel
            if (self.actual_position > self.set_pos):
                self.actual_position = min(self.actual_position + movement, self.set_pos)
            if (self.actual_position < self.set_pos):
                self.actual_position = max(self.actual_position - movement, self.set_pos)
        else:
            movement = self.deltaTime() * self.set_move_vel
            self.actual_position += self.deltaTime() * self.set_move_vel
            
        self.last_update = time.time()
        
    def debug(self):
        return f"act: {self.actual_position} pos: {self.set_pos} vel: {self.set_vel} move: {self.set_move_vel} acc: {self.set_acc}"
        

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
                ('mock_servos', rclpy.Parameter.Type.BOOL),
                ('bitrate', rclpy.Parameter.Type.INTEGER),
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
        self._ySpeed = self.get_parameter_or('tpv_y_speed', 5000)
        
        self.mock = self.get_parameter_or('mock_servos', False)
        self.bitrate = self.get_parameter_or('bitrate', CANBUS_BITRATE)
        
        if self.mock:
            self.motor_x = ServoMock(self._xSpeed/2)
            self.motor_y = ServoMock(self._ySpeed/2)
            self.get_logger().info("Mock enabled")
        
        # Setup pubs and subs
        self.servo_pos = self.create_publisher(Vector3Stamped, 'servo_pos', 0)
        self.servo_set_pos = self.create_subscription(Vector3Stamped, 'servo_set_pos', self.setPos, 3)
        self.servo_set_vel = self.create_subscription(Vector3Stamped, 'servo_set_vel', self.setVel, 3)
        self.servo_set_acc = self.create_subscription(Vector3Stamped, 'servo_set_acc', self.setAcc, 3)
        self.servo_set_move_vel = self.create_subscription(Vector3Stamped, 'servo_set_move_vel', self.setMoveVel, 3)
        # self.servo_set_mode = self.create_subscription(UInt32, 'servo_set_mode', self.setMode, 3)
        # self.servo_joy = self.create_subscription(Joy, '/joy', self.joy, 0)
        self.led_front = self.create_publisher(Bool, 'led_front', 0)
        self.led_front_set = self.create_subscription(Bool, 'led_front_set', self.setLEDFront, 3)
        self.led_back = self.create_publisher(Bool, 'led_back', 0)
        self.led_back_set = self.create_subscription(Bool, 'led_back_set', self.setLEDBack, 3)
        self.led_strobe = self.create_publisher(Bool, 'led_strobe', 0)
        self.led_strobe_set = self.create_subscription(Bool, 'led_strobe_set', self.setLEDStrobe, 3)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        
        self._control_mode = api.ServoControlMode.SCMPosition
        self._control_mode_timeout = 5
        self._control_mode_return = 0
        
        # Setup status report
        self.heartbeat = self.create_timer(1/5, self.statusReport)
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
        self._set_vel = [1000,1000]
        self._move_vel = [0,0]
        self._set_acc = [150,150]
        self._connected = False
    
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
        from rove_control_board.api import manager
        if self.mock:
            return True
        if (not manager.apiCheck()):
            self.get_logger().fatal("API hash mismatch between rove_control_board and control board microcontroller")
            return False
        return True
    
    def _xToRad(self, step:int) -> float:
        # return toDeg(step, self._xStepCount, self._xOffset, self._xMin, self._xMax)
        return deg2rad(toDeg(step, self._xStepCount))
    
    def _xToStep(self, rad:float) -> int:
        # return toStep(deg, self._xStepCount, self._xOffset, self._xMin, self._xMax)
        return toStep(rad2deg(rad), self._xStepCount)
    
    def _yToRad(self, step:int) -> float:
        # return toDeg(step, self._yStepCount, self._yOffset, self._yMin, self._yMax)
        return deg2rad(toDeg(step, self._yStepCount))
    
    def _yToStep(self, rad:float) -> int:
        # return toStep(deg, self._yStepCount, self._yOffset, self._yMin, self._yMax)
        return toStep(rad2deg(rad), self._yStepCount)
    
    @property
    def nowSeconds(self):
        return time.time()
        # return self.get_clock().now().nanoseconds / 1000000000.0
    
    def _pushbackControlTimeout(self):
        self._control_mode_return = self.nowSeconds + self._control_mode_timeout
        if self._control_mode == api.ServoControlMode.SCMPosition:
            self.setMode(api.ServoControlMode.SCMSpeed)
    
    def _checkControlTimeout(self):
        if self._control_mode == api.ServoControlMode.SCMSpeed and self._control_mode_return < self.nowSeconds:
            self.setMode(api.ServoControlMode.SCMPosition)
            return True
        return self._control_mode == api.ServoControlMode.SCMPosition
    
    @debugFunc
    def joy(self, value:Joy):
        x = value.axes[3]
        y = value.axes[4]
        if x == self._move_vel[0] and y == self._move_vel[1]:
            return
        self._move_vel = [x, y]
        vel = Vector3Stamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        vel.header.frame_id = 'servo_move_vel'
        vel.vector.x = x
        vel.vector.y = y
        self.setMoveVel(vel)
    
    @debugFunc
    def setPos(self, pos:Vector3Stamped):
        if not self._checkControlTimeout():
            return
        try:
            if self.mock:
                position = (self._xToStep(pos.vector.x), self._yToStep(pos.vector.y))
                self.motor_x.move_pos(position[0])
                self.motor_y.move_pos(position[1])
                return
            r = api.setServoPosition(api.Vector2D(*position))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setPos)")
        except ValueError as e:
            self.get_logger().error(str(e))
    
    @debugFunc
    def setVel(self, vel:Vector3Stamped):
        if not self._checkControlTimeout():
            return
        try:
            if self.mock:
                self.motor_x.set_vel = vel.vector.x
                self.motor_y.set_vel = vel.vector.y
                return
            r = api.setServoSpeed(api.Vector2D(int(vel.vector.x), int(vel.vector.y)))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setVel)")
        except ValueError as e:
            self.get_logger().error(str(e))
        
    @debugFunc
    def setMoveVel(self, vel:Vector3Stamped):
        try:
            if self.mock:
                self.motor_x.move_vel(vel.vector.x)
                self.motor_y.move_vel(vel.vector.y)
                return
            r = api.setServoSpeed(api.Vector2D(int(vel.vector.x * self._xSpeed), int(vel.vector.y * self._ySpeed)))
            self._pushbackControlTimeout()
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setMoveVel)")
        except ValueError as e:
            self.get_logger().error(str(e))
            
    @debugFunc
    def setAcc(self, acc:Vector3Stamped):
        try:
            if self.mock:
                self.motor_x.set_acc = acc.vector.x
                self.motor_y.set_acc = acc.vector.y
                return
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
        if self.mock:
            self._control_mode = mode
            self.get_logger().info(f"Switched to {mode}")
            return
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setMode)")
        else:
            self._control_mode = mode
          
    @debugFunc
    def getMode(self):
        if self.mock:
            return self._control_mode
        r = api.getServoControlMode()
        try:
            return api.ServoControlMode(r.s)
        except ValueError as e:
            self.get_logger().error(e)
        
    @debugFunc
    def setLEDFront(self, state:Bool):
        if self.mock:
            return
        r = api.setLEDFront(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDFront)")
    
    @debugFunc
    def setLEDBack(self, state:Bool):
        if self.mock:
            return
        r = api.setLEDBack(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDBack)")
    
    @debugFunc
    def setLEDStrobe(self, state:Bool):
        if self.mock:
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
    def getPos(self) -> Vector3Stamped:
        r = Vector3Stamped()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = 'getPos'
        r.vector.x = self._xToRad(self._pos[0])
        r.vector.y = self._yToRad(self._pos[1])
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
        if self.mock:
            self.motor_x.update()
            self.motor_y.update()
            self._pos[0] = self.motor_x.actual_position
            self._pos[1] = self.motor_y.actual_position
            self.get_logger().info(f"Motor x: {self.motor_x.debug()}")
            self.get_logger().info(f"Motor y: {self.motor_y.debug()}")
        else:
            r = api.getReport(comm.Void())

            if r.errorCode != api.ErrorCode.ERNone.value and not self.mock:
                try:
                    self.get_logger().error(f"Got {api.ErrorCode(r.errorCode).name} from report.")
                except ValueError as e:
                    self.get_logger().error(str(e))
            
            self._pos[0] = r.pos.x
            self._pos[1] = r.pos.y
            
        pos = self.getPos()
        self.servo_pos.publish(pos)
        states = JointState()
        states.name = ['tpv_x_joint', 'tpv_y_joint']
        states.header.stamp = pos.header.stamp
        states.header.frame_id = ''
        states.position = [pos.vector.x, pos.vector.y]
        states.velocity = [0.0, 0.0]
        states.effort = [0.0, 0.0]
        self.get_logger().info(f"{pos.vector.x} {pos.vector.y}")
        self.joint_state_pub.publish(states)
        self.led_front.publish(self.getLEDFront())
        self.led_back.publish(self.getLEDBack())
        self.led_strobe.publish(self.getLEDStrobe())
        
    @debugFunc
    def ping(self):
        from rove_control_board.api import manager
        p = manager.ping()
        self.get_logger().info(f"Ping: {round(p*1000, 2)} ms")
        
def openSocket(bitrate:int, logger:rclpy.impl.rcutils_logger.RcutilsLogger):
    from rove_control_board.api import manager
    
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
    from rove_control_board.api import manager
    
    # Init ros and bridge
    rclpy.init(args=args)
    bridge = Bridge()
    logger = bridge.get_logger()
        
    if isinstance(manager, comm.SerialCommandManager):
        # Serial comm 
        manager.port = DEV
        manager.baud = 9600
        manager._stream.rts = True
        manager._stream.open()
    elif isinstance(manager, canutils.CanBusCommandManager):
        # CAN comm
        if not bridge.mock:
            openSocket(bridge.bitrate, logger)
    
    try:
        bridge.connect()
        rclpy.spin(bridge)
        rclpy.shutdown()
    except KeyboardInterrupt:
        rclpy.try_shutdown()

if __name__ == '__main__':
    
    main()