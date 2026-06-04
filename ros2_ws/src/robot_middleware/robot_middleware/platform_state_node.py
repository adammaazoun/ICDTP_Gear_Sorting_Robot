#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
from std_msgs.msg import Float32


class PlatformStateNode(Node):

    SERVO_PIN = 18

    def __init__(self):
        super().__init__('platform_state_node')

        # ROS interfaces
        self.cmd_sub = self.create_subscription(
            Float32,
            '/inspection_servo_cmd',
            self.cmd_callback,
            10
        )

        self.state_pub = self.create_publisher(
            Float32,
            '/inspection_servo_state',
            10
        )

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(self.SERVO_PIN, 50)
        self.pwm.start(0)

        self.current_angle = 0.0

        self.get_logger().info("Platform State Node started")

    # ─────────────────────────────────────
    # Receive command from Unity / ROS
    # ─────────────────────────────────────
    def cmd_callback(self, msg: Float32):

        target = msg.data
        self.get_logger().info(f"Servo target: {target}°")

        self.set_angle(target)

        # update internal state
        self.current_angle = target

        # publish state back
        self.publish_state()

    # ─────────────────────────────────────
    # Hardware control
    # ─────────────────────────────────────
    def set_angle(self, angle):

        duty = 2 + angle / 18
        self.pwm.ChangeDutyCycle(duty)

    # ─────────────────────────────────────
    # Publish state to Unity
    # ─────────────────────────────────────
    def publish_state(self):

        msg = Float32()
        msg.data = float(self.current_angle)
        self.state_pub.publish(msg)

    def destroy_node(self):

        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlatformStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()