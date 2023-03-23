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
speed_change= 0.05
front_angle = 30
front_angle_range = range(-front_angle,front_angle+1,1)
stop_distance = 0.25
waypoint = 0

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
        
        #self.timer_period = 0.5
        
        #self.timer = self.create_timer(self.timer_period, self.motion)

        self.cmd = Twist()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.laser_range = np.array([])
        self.laser_forward = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0

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
        self.get_logger().info('I receive "%s"' % str(self.cmd.angular.z))
        self.publisher_.publish(self.cmd)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
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

    def MoveForward(self, y_coord):
        
        self.get_logger().info('I receive "%s"' % str(self.odom_y))
        
        while (self.odom_y < y_coord):
            rclpy.spin_once(self)
            self.get_logger().info('I receive "%s"' % str(self.odom_y))
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
         
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)

    def MoveBackwards(self, y_coord):
        
        self.get_logger().info('I receive "%s"' % str(self.odom_y))
        
        while (self.odom_y > y_coord):
            rclpy.spin_once(self)
            self.get_logger().info('I receive "%s"' % str(self.odom_y))
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
         
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)

    def MoveRight(self, x_coord):

        self.get_logger().info('I receive "%s"' % str(self.odom_x))

        
        while (self.odom_x < x_coord):
            rclpy.spin_once(self)
            self.get_logger().info('I receive "%s"' % str(self.odom_x))
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
         
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)

    def MoveLeft(self, x_coord):
        self.get_logger().info('I receive "%s"' % str(self.odom_x))
        
        while (self.odom_x > x_coord):
            rclpy.spin_once(self)
            self.get_logger().info('I receive "%s"' % str(self.odom_x))
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
         
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)

    def TurnRight(self):
        
        self.get_logger().info('Turning Right Now!')
        self.rotatebot(-90.0)

    def TurnLeft(self):
        
        self.get_logger().info('Turning Left Now!')
        self.rotatebot(90.0)

    def Turn180(self):
        
        self.get_logger().info('Turning Back Now!')
        self.rotatebot(180.0)

    def motion_callback(self):
        self.get_logger().info('I receive "%s"' % str(self.odom_x))

    def travel_to_waypoint(self, waypoint_num):
        if waypoint_num == 8:
            
            self.MoveForward(1.85)


    def motion(self):
        
        try:
            waypoint_arr = np.genfromtxt('/home/nicholas/colcon_ws/src/auto_nav/auto_nav/waypoint_log.txt', delimiter=' ')
            # print(waypoint_arr)
            while rclpy.ok():
                
                table_num = 0
                while table_num < 1 or table_num > 6:
                    table_num = int(input("Enter a table number to deliver to: "))
                    
                # all these are the coordinates of the table where the turtlebot will be stopping at!
                if (table_num == 1):
                    self.get_logger().info('Moving to table "%s"!' % str(table_num))
                    self.travel_to_waypoint(8)

                    # self.MoveForward(1.85)
                
                elif (table_num == 2):
                    self.get_logger().info('Moving to table "%s"!' % str(table_num))
                    self.MoveForward(1.67)
                    self.MoveRight(1.35)
                
                elif (table_num == 3):
                    self.get_logger().info('Moving to table "%s"!' % str(table_num))
                    self.MoveForward(1.01)
                    self.MoveRight(0.96)

                elif (table_num == 4):
                    self.get_logger().info('Moving to table "%s"!' % str(table_num))
                    self.MoveForward(0.48)
                    self.MoveRight(2.13)

                elif (table_num == 5):
                    self.get_logger().info('Moving to table "%s"!' % str(table_num))
                    self.MoveForward(0.48)
                    self.MoveRight(2.96)
                    self.MoveLeftUp(1.86)

                else: # table_num == 6
                    self.get_logger().info('Moving to table "%s"!' % str(table_num))
                    self.MoveForward(1.67)
                    self.MoveRight(2.03)
                    self.MoveLeftUp(3.45)
                    self.MoveLeft(1.08)
        
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
    hardcoded_navi_node.motion()
    hardcoded_navi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
