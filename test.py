
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import time
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import math
import cmath

rotate_change = 0.35
slow_rotate_change = 0.10
front_angle = 3
front_angle_6 = 80

def euler_from_quaternion(quaternion): 
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

        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            'scan', 
            self.laser_callback, 
            qos_profile=qos_profile_sensor_data)

        self.yaw = 0.0
        
    def map2base_callback(self, msg):
        
        orientation_quat = msg.orientation
        quaternion = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)
        self.mapbase = msg.position
        self.x_coordinate = msg.position.x
        self.y_coordinate = msg.position.y
    

    def Clocking(self):
        time_started = self.get_clock().now().to_msg().sec
        timee = 0
        while self.x_coordinate <= 10.0:
            self.cmd.linear.x = 0.1
            self.publisher_.publish(self.cmd)
            rclpy.spin_once()
            time_diff = self.get_clock().now().to_msg().sec - time_started
            timee = self.get_clock().now().to_msg().nanosec/float(1000000000) + time_diff
            speed = self.x_coordinate / timee
            print(speed) 
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

    def rotatebot(self, rot_angle):
        self.get_logger().info('In rotatebot')
        
        rotation_speed = 0
        if abs(rot_angle) < 30:
            rotation_speed = slow_rotate_change
        else:
            rotation_speed = rotate_change
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        # self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        # self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        self.cmd.linear.x = 0.0
        # set the direction to rotate
        self.cmd.angular.z = c_change_dir * rotation_speed
        # start rotation
        # self.get_logger().info('I receive "%s"' % str(self.cmd.angular.z))
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

            print(current_yaw)

        # self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        self.cmd.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(self.cmd)



    def laser_callback(self, msg):

        self.laser_range = np.array(msg.ranges[0:front_angle] + msg.ranges[-front_angle:])
        self.laser_range_6 = np.array(msg.ranges[0:front_angle_6] + msg.ranges[-front_angle_6:])
        self.laser_range[self.laser_range==0] = np.nan
        self.laser_range_6[self.laser_range_6==0] = np.nan
        self.laser_angle_increment = msg.angle_increment
        self.laser_forward = np.nanmean(self.laser_range) / 2
        self.laser_forward_6 = np.nanmean(self.laser_range_6) / 2

    def testing(self):
        self.rotatebot(math.degrees(-self.yaw))  
    
    def stopbot(self, delay):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        time.sleep(delay)

    def move_robot(self):
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
                self.rotatebot(40)
                self.stopbot(0.1)
                self.rotatebot(40)
                self.stopbot(0.1)
            # time.sleep(3)   
            except Exception as e:
                print(e)

            # Ctrl-c detected
            finally:
                # stop moving
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)



def main(args=None):

    rclpy.init(args=args)
    hardcoded_navi_node = Navigation()
    #hardcoded_navi_node.Clocking()
    hardcoded_navi_node.move_robot()
    #hardcoded_navi_node.testing() 
    hardcoded_navi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        