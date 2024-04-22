from typing import NamedTuple, NoReturn, Tuple

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

DEV = '/dev/ttyACM0'

# TPV bounds ((horiz min, horiz max), (verti min, verti max))
TPV_BOUNDS:Tuple[Tuple[float, float], Tuple[float,float]] = ((-180, 180), (-45, 90))
MAX_SPEED_X = 10000
MAX_SPEED_Y = 10000

@api.setServoPosition.preCall
def setServoPositionValidator(pos:api.Vector2D) -> NoReturn:
    if TPV_BOUNDS[0][0] <= pos[0] <= TPV_BOUNDS[0][1] and TPV_BOUNDS[1][0] <= pos[1] <= TPV_BOUNDS[1][1]:
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

class Bridge(Node):
    def __init__(self):
        super().__init__('control_board_bridge', namespace='control_board_bridge')
        self.rate = self.create_rate(100)
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
        
        self._led_front = False
        self._led_back = False
        self._led_strobe = False
        self._pos = [0,0]
        self._set_pos = [0,0]
        self._set_vel = [0,0]
        self._set_acc = [0,0]
        self._connected = False
        
        
    def setPos(self, pos:Vector3Stamped):
        try:
            r = api.setServoPosition(api.Vector2D(pos.vector.x, pos.vector.y))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setPos)")
        except ValueError as e:
            self.get_logger().error(e)
    
    def setVel(self, vel:Vector3Stamped):
        try:
            r = api.setServoSpeed(api.Vector2D(vel.vector.x, vel.vector.y))
            if not r.b:
                self.get_logger().warning(f"Last command didn't ack (from setVel)")
        except ValueError as e:
            self.get_logger().error(e)
            
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
    
    def setMode(self, mode:UInt32):
        raise NotImplementedError()
        
    def setLEDFront(self, state:Bool):
        r = api.setLEDFront(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDFront)")
    
    def setLEDBack(self, state:Bool):
        r = api.setLEDBack(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDBack)")
    
    def setLEDStrobe(self, state:Bool):
        r = api.setLEDStrobe(comm.Bool_(state.data))
        if not r.b:
            self.get_logger().warning(f"Last command didn't ack (from setLEDStrobe)")
    
    def getPos(self) -> Vector3Stamped:
        r = Vector3Stamped()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = 'getPos'
        r.vector.x = self._pos[0]
        r.vector.y = self._pos[1]
        return r
            
    def getLEDFront(self) -> Bool:
        return Bool(self._led_front)
    
    def getLEDBack(self) -> Bool:
        return Bool(self._led_back)
    
    def getLEDStrobe(self) -> Bool:
        return Bool(self._led_strobe)
    
    def statusReport(self):
        r = api.getReport(comm.Void())
        if r.errorCode != api.ErrorCode.ERNone:
            self.get_logger().error(f"Got {api.ErrorCode(r.errorCode).name} from report.")
        
        self.servo_pos.publish(self.getPos())
        self.led_front.publish(self.getLEDFront())
        self.led_back.publish(self.getLEDBack())
        self.led_strobe.publish(self.getLEDStrobe())
        
def main(args=None):
    from rove_control_board.api import manager
    
    print(manager.generateAPI())
    
    if isinstance(manager, comm.SerialCommandManager):
        # Serial comm
        manager.port = DEV
        manager.baud = 9600
        manager._stream.rts = True
        manager._stream.open()
    elif isinstance(manager, canutils.CanBusCommandManager):
        # CAN comm
        # manager.interface = ...
        # manager.channel = ...
        pass
    
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