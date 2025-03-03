#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

ENA = 13
ENB = 12
IN1 = 1
IN2 = 7
IN3 = 8
IN4 = 25

PWM_FREQ = 1000  # 1 kHz PWM frequency

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('Motor Control Node has started')

        GPIO.setmode(GPIO.BCM)
        GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
        GPIO.setup([ENA, ENB], GPIO.OUT)

        self.pwm_a = GPIO.PWM(ENA, PWM_FREQ)
        self.pwm_b = GPIO.PWM(ENB, PWM_FREQ)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        speed = abs(int(linear * 100))  # Convert speed to percentage (0-100)
        turn = abs(int(angular * 100))

        if linear > 0:  # Forward
            self.set_motor(True, speed, True, speed)
        elif linear < 0:  # Backward
            self.set_motor(False, speed, False, speed)
        elif angular > 0:  # Turn left
            self.set_motor(False, turn, True, turn)
        elif angular < 0:  # Turn right
            self.set_motor(True, turn, False, turn)
        else:  # Stop
            self.set_motor(False, 0, False, 0)

    def set_motor(self, dir_a, speed_a, dir_b, speed_b):
        # Motor A
        GPIO.output(IN1, GPIO.HIGH if dir_a else GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW if dir_a else GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(speed_a)

        # Motor B
        GPIO.output(IN3, GPIO.HIGH if dir_b else GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW if dir_b else GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(speed_b)

    def destroy_node(self):
        self.get_logger().info("Shutting down, stopping motors")
        self.set_motor(False, 0, False, 0)
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
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