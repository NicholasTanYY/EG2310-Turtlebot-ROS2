#!/usr/bin/env python3

import serial
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BeepNode(Node):

    def __init__(self):
        super().__init__('beep_node')

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 57600

        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        self.subscription_ = self.create_subscription(
            Bool,
            'beep_topic',
            self.beep_callback,
            10)

    def beep_callback(self, msg):
        if msg.data:
            self.ser.write(b'1')
            self.get_logger().info('Beeped')

def main(args=None):
    rclpy.init(args=args)

    node = BeepNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

