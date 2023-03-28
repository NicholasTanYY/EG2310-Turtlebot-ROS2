import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from custom_msgs.msg import Button

button_pin = 

GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

class ButtonPress(Node):

    def __init__(self):
        super().__init__('button')
        self.publisher_ = self.create_publisher(Button, 'button_pressed', 10)
        self.get_logger().info('Created publisher')
        self.get_logger().info('Button function started')
        # self.button_callback()

    def button_callback(self):
        try:
            
            msg = Button()
            self.button_pressed = False

            while True:
                if GPIO.input(button_pin) != GPIO.LOW:
                    self.get_logger().info("Button not pressed")
                    continue

                self.get_logger().info('Button pressed')
                msg.button_pressed = True
                self.publisher_.publish(msg)
                break
                
        except Exception as e:
            print(e)
        #finally:
        #    GPIO.cleanup()