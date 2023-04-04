import rclpy
from std_msgs.msg import Bool

def button_callback(msg):
    if msg.data:
        print("Button pressed!")
    else:
        print("Button released.")

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('button_subscriber')
    subscriber = node.create_subscription(Bool, 'button_pressed', button_callback, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()