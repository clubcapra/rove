import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Bool, ColorRGBA
from sensor_msgs.msg import Temperature



class Bridge(Node):
    def __init__(self):
        super().__init__('control_board_bridge')
        


def main(args=None):
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