#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

instructions = """
Control Your Robot!
---------------------------
Moving around:
   w
a  s  d

w - forward
s - stop
x - backward
a - turn left
d - turn right
q - quit
"""

def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Teleop Keyboard Node started")
        print(instructions)

    def run(self):
        try:
            while True:
                key = get_key()
                twist = Twist()
                if key == 'w':
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0
                elif key == 'x':
                    twist.linear.x = -0.5
                    twist.angular.z = 0.0
                elif key == 'a':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                elif key == 'd':
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
                elif key == 's':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == 'q':
                    break
                else:
                    continue
                self.publisher_.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
