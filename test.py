
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import time
from geometry_msgs.msg import Pose
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
from geometry_msgs.msg import Twist

class Navigation(Node):
    
    def __init__(self):

        super().__init__('hardcoded_navi')

        self.publisher_ = self.create_publisher(
                Twist, 
                'cmd_vel', 
                10)
        
        self.map2base_subscriber = self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callback,
            1)
    
        self.cmd = Twist()

    def Clocking(self):
        time_started = self.get_clock().now().to_msg().sec
        timee = 0
        while self.x_coordinate <= 10.0:
            self.cmd.linear.x = 0.1
            self.publisher_.publish(self.cmd)
            rclpy.spin_once()
            speed = self.x_coordinate / time
            print(speed) 
            time_diff = self.get_clock().now().to_msg().sec - time_started
            timee = self.get_clock().now().to_msg().nanosec/float(1000000000) + time_diff
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)


def main(args=None):

    rclpy.init(args=args)
    hardcoded_navi_node = Navigation()
    hardcoded_navi_node.Clocking()
    time.sleep(3)
    hardcoded_navi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        