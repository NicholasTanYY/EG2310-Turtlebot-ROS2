# import rclpy
# from rclpy.node import Node
# import RPi.GPIO as GPIO
# import time
# from custom_msgs.msg import Button

# button_pin = 15

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# class ButtonPress(Node):

#     def __init__(self):
#         super().__init__('button')
#         self.publisher_ = self.create_publisher(Button, 'button_pressed', 10)
#         self.get_logger().info('Created publisher')
#         self.get_logger().info('Button function started')
#         # self.button_callback()

#     def button_callback(self):
#         try:
            
#             msg = Button()
#             self.button_pressed = False

#             while True:
#                 if GPIO.input(button_pin) != GPIO.LOW:
#                     self.get_logger().info("Button not pressed")
#                     continue

#                 self.get_logger().info('Button pressed')
#                 msg.button_pressed = True
#                 self.publisher_.publish(msg)
#                 break
                
#         except Exception as e:
#             print(e)
#         #finally:
#         #    GPIO.cleanup()


# Code to be placed into RPi
# Run by keying in 'python3 button.py' into terminal
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
import rclpy

button_pin = 15

def button_pressed(channel):
    # This function will be called when the button is pressed
    # Publish a ROS 2 message indicating that the button was pressed
    msg = Bool()
    msg.data = True
    print("Button pressed!")
    publisher.publish(msg)

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_pressed, bouncetime=50)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('button_input_node')
    global publisher
    publisher = node.create_publisher(Bool, 'button_pressed', 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
