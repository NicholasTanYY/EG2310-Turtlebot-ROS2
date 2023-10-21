import RPi.GPIO as GPIO
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ButtonInputNode(Node):

    def __init__(self):
        super().__init__('button_input_node')
        
        self.button_pin = 18

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.publisher_ = self.create_publisher(Bool, 'button_pressed', 10)
        
        self.timer_ = self.create_timer(0.1, self.check_button_press)
        self.get_logger().info('Ready for button press!')

    def check_button_press(self):
        if GPIO.input(self.button_pin) == GPIO.LOW:
            self.get_logger().info('Button pressed')
            msg = Bool()
            msg.data = True
            self.publisher_.publish(msg)
            time.sleep(0.2)
        else:
            self.get_logger().info('Button released')
            msg = Bool()
            msg.data = False
            self.publisher_.publish(msg)
            time.sleep(0.2)

def main(args=None):
    print('Preparing setup... please wait')
    rclpy.init(args=args)

    node = ButtonInputNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

