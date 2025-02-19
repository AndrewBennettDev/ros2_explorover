#!/usr/bin/env python3
import rclpy
from rclply.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# TODO: check port
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE= 9600

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.subscription = self.create_subscription(
                Twist,
                'cmd_vel',
                self.cmd_vel_callback,
                10)
        self.get_logger().info('Motor Control Node has started')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # TODO: wait for resp from nano?
            self.get_logger().info(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        if self.ser is None:
            self.get_logger().error("Serial port not open")
            return

        linear = msg.linear.x
        angular = msg.angular.z

        if linear > 0:
            command = 'F' # forward
        elif linear < 0:
            command = 'B' # backward
        elif angular > 0:
            command = 'L' # left
        elif angular < 0:
            command = 'R' # right
        else:
            command = 'E' # stop

        self.get_logger().debug(f"Sending command: {command}")
        try:
            self.ser.write(command.encode())
        except Exception as e:
            self.get_logger().error(f"Error writing to serial: {e}")


    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
