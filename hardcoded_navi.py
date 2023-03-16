import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import numpy as np
import cmath
import time

rotate_change = 0.1
speed_change=0.05
front_angle = 30
front_angle_range = range(-front_angle,front_angle+1,1)
stop_distance = 0.25

class Navigation(Node):
    
    def __init__(self):

        super().__init__('hardcoded_navi')

        self.publisher_ = self.create_publisher(
                Twist, 
                'cmd_vel', 
                10)
        
        self.odom_subscriber = self.create_subscription(
                Odometry, 
                'odom', 
                self.odom_callback, 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.scan_subscriber = self.create_subscription(
                LaserScan, 
                'scan', 
                self.laser_callback, 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.timer_period = 0.5
        
        self.timer = self.create_timer(self.timer_period, self.motion)

        self.cmd = Twist()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.laser_range = np.array([])
        self.laser_forward = 0.0

    def euler_from_quaternion(self, quaternion): 
        """ 
        Converts quaternion (w in last place) to euler roll, pitch, yaw 
        quaternion = [x, y, z, w] 
        Below should be replaced when porting for ROS2 Python tf_conversions is done. 
        """ 
        x = quaternion[0] 
        y = quaternion[1] 
        z = quaternion[2] 
        w = quaternion[3] 
 
        sinr_cosp = 2 * (w * x + y * z) 
        cosr_cosp = 1 - 2 * (x * x + y * y) 
        roll = np.arctan2(sinr_cosp, cosr_cosp) 
 
        sinp = 2 * (w * y - z * x) 
        pitch = np.arcsin(sinp) 
 
        siny_cosp = 2 * (w * z + x * y) 
        cosy_cosp = 1 - 2 * (y * y + z * z) 
        yaw = np.arctan2(siny_cosp, cosy_cosp) 
 
        return roll, pitch, yaw
    
    def odom_callback(self, msg):
        
        orientation_quat = msg.pose.pose.orientation
        quaternion = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion(quaternion)
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def laser_callback(self, msg):

        self.laser_range = np.array(msg.ranges)

        self.laser_range[self.laser_range==0] = np.nan


    def stopbot(self):

        if self.laser_range.size != 0: 
            
            lri = (self.laser_range[front_angle_range]<float(stop_distance)).nonzero()
            if(len(lri[0])>0):
                self.get_logger().info('Stopping the bot!') 
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)

    
    def rotatebot(self, rot_angle):
        self.get_logger().info('In rotatebot')
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        self.cmd.linear.x = 0.0
        # set the direction to rotate
        self.cmd.angular.z = c_change_dir * rotate_change
        # start rotation
        self.publisher_.publish(self.cmd)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        self.cmd.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(self.cmd)

    def MoveForward(self, distance):
        
        self.get_logger().info('I receive "%s"' % str(self.odom_y))
        
        while (self.odom_y < distance):
            rclpy.spin_once(self)
            self.get_logger().info('I receive "%s"' % str(self.odom_y))
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
         
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)


    def MoveRight(self, distance):

        self.get_logger().info('I receive "%s"' % str(self.odom_x))

        self.rotatebot(270.0)
        while (self.odom_x < distance):
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
         
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)

    def MoveLeft(self, distance):
        self.get_logger().info('I receive "%s"' % str(self.odom_x))
        
        self.rotatebot(90.0)
        while (self.odom_x < distance):
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
         
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)

    def motion_callback(self):
        self.get_logger().info('I receive "%s"' % str(self.odom_x))

    def motion(self):
        
        distance = 1.22
        if (FirstMoveDone == False):
            if (self.odom_y < distance):
                self.get_logger().info('I receive "%s"' % str(self.odom_y))
                self.cmd.linear.x = speed_change
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
            else:
                self.cmd.linear.x = 0.0
                self.publisher_.publish(self.cmd)
                FirstMoveDone = True
                SecondMoveDone = False

        if (SecondMoveDone == False):
            self.rotatebot(270.0)
                        

        if 
        self.MoveForward(1.22)
        self.MoveRight(0.6)

def main(args=None):

    rclpy.init(args=args)
    hardcoded_navi_node = Navigation()
    rclpy.spin(hardcoded_navi_node)
    hardcoded_navi_node.motion()
    hardcoded_navi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

