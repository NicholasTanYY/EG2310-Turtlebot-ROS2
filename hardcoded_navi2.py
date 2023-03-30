import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import math
import numpy as np
import cmath
import time
from PIL import Image
import scipy.stats
import matplotlib.pyplot as plt

rotate_change = 0.15
speed_change= 0.15
front_angle = 30
front_angle_range = range(-front_angle,front_angle+1,1)
stop_distance = 0.25
occ_bins = [-1, 0, 50, 101]
box_thres = 0.1
f_path = '/home/nicholas/colcon_ws/src/auto_nav/auto_nav/confirmed_waypoints.txt'

def calculate_yaw_and_distance(x1, y1, x2, y2, current_yaw):
    """
    Calculates the yaw the robot needs to turn to and the distance it needs to travel to move from point (x1, y1)
    to point (x2, y2) on a 2D plane, given its current yaw coordinate.
    Returns a tuple containing the new yaw and the distance as float values.
    """
    # Calculate the angle between the two points using the arctan2 function
    delta_x = x2 - x1
    delta_y = y2 - y1
    target_yaw = math.atan2(delta_y, delta_x)

    # Calculate the distance between the two points using the Pythagorean theorem
    distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

    # Calculate the difference between the current yaw and the target yaw

    print("target_yaw = ", target_yaw)
    print("current_yaw = ", current_yaw)
    yaw_difference = target_yaw - current_yaw

    # Normalize the yaw difference to between -pi and pi radians
    if yaw_difference > math.pi:
        yaw_difference -= 2 * math.pi
    elif yaw_difference < -math.pi:
        yaw_difference += 2 * math.pi

    return (round(yaw_difference, 3), round(distance, 3))

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
        
        self.scan_subscriber = self.create_subscription(
                LaserScan, 
                'scan', 
                self.laser_callback, 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            1)

        self.mqtt_subscription = self.create_subscription(
            String,
            'mqtt_data',
            self.mqtt_callback,
            1)

        self.cmd = Twist()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.laser_range = np.array([])
        self.laser_forward = 0.0
        self.mapbase = Pose().position
        self.Xpos = 0
        self.Ypos = 0
        self.XposNoAdjust = 0
        self.YposNoAdjust = 0
        self.mazelayout = []
        self.visitedarray = np.zeros((300,300),int)
        self.previousaction = []
        self.resolution = 0.05
        self.Xadjust = 0
        self.Yadjust = 0
        self.waypoint_arr = np.genfromtxt(f_path)
        self.mqtt_val = 0
    
    def map2base_callback(self, msg):
        
        orientation_quat = msg.orientation
        quaternion = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)
        self.mapbase = msg.position
        self.Xpos = msg.position.x
        self.Ypos = msg.position.y

    def laser_callback(self, msg):

        self.laser_range = np.array(msg.ranges)

        self.laser_range[self.laser_range==0] = np.nan

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 50. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        # calculate total number of bins
        total_bins = iwidth * iheight
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        self.mazelayout = np.uint8(binnum.reshape(msg.info.height,msg.info.width))

        self.Xpos = int(np.rint((self.mapbase.x - msg.info.origin.position.x)/self.resolution))
        self.Ypos = int(np.rint((self.mapbase.y - msg.info.origin.position.y)/self.resolution))
        self.mazelayout[self.Ypos][self.Xpos] = 6
        # self.mazelayout[self.Ypos][self.Xpos - 5] = 10
        self.XposNoAdjust = int(np.rint((self.mapbase.x + 5)/self.resolution))
        self.YposNoAdjust = int(np.rint((self.mapbase.y + 5)/self.resolution))
        self.Xadjust = msg.info.origin.position.x
        self.Yadjust = msg.info.origin.position.y
        img2 = Image.fromarray(self.mazelayout)
        img = Image.fromarray(np.uint8(self.visitedarray.reshape(300,300)))
        plt.imshow(img2, cmap='gray', origin='lower')
        plt.draw_all()
        # # pause to make sure the plot gets created
        plt.pause(0.00000000001)

    def mqtt_callback(self, msg):
        # print("before decode", msg.data)
        # data = msg.data.decode('UTF-8') # str has no attribute decode
        data = msg.data
        print("Received MQTT data:", data)
        self.mqtt_val = int(data)
        
    def stopbot(self, delay):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        time.sleep(delay)

    
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

    def MoveForward(self, x_coord, y_coord):
        
        self.get_logger().info('Moving Forward...')
        while not ((self.mapbase.y < y_coord + box_thres and self.mapbase.y > y_coord - box_thres) and (self.mapbase.x < x_coord + box_thres and self.mapbase.x > x_coord - box_thres)):
            rclpy.spin_once(self)
            # self.get_logger().info('I receive "%s"' % str(self.mapbase.y))
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
        
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)
    
    def move_to_waypoint(self, WP_num):
        x1, y1, x2, y2, current_yaw = self.Xpos, self.Ypos, self.waypoint_arr[WP_num][0], self.waypoint_arr[WP_num][1], self.yaw
        yaw_difference, distance = calculate_yaw_and_distance(x1, y1, x2, y2, current_yaw)
        yaw_difference = yaw_difference / math.pi * 180

        print(f"To reach the point ({x2}, {y2}), the robot needs to turn {yaw_difference} degrees and travel {distance} units.")

        self.rotatebot(yaw_difference)
        self.stopbot(0.1)

        self.MoveForward(x2, y2)
        self.stopbot(0.1)

        self.get_logger().info('Waypoint reached!')

    def motion(self):
        
        try:
            while rclpy.ok():
                
                rclpy.spin_once(self)
                print("current yaw = ", self.yaw)
                print("current Xpos = ", self.Xpos)
                print("current Ypos = ", self.Ypos)

                print("Waiting for mqtt input...")
                while self.mqtt_val == 0:
                    rclpy.spin_once(self)
                
                table_num = self.mqtt_val
                print("table_num received = ", table_num)

                if (table_num == 1):
                    # moving to the table
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(7)
                    self.move_to_waypoint(8)
                    self.move_to_waypoint(9)
                    self.move_to_waypoint(15)

                    # moving back to the dispenser
                    self.move_to_waypoint(9)
                    self.move_to_waypoint(8)
                    self.move_to_waypoint(7)
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(1)


                elif (table_num == 2):
                    # moving to the table
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(7)
                    self.move_to_waypoint(8)
                    self.move_to_waypoint(9)
                    self.move_to_waypoint(10)
                    self.move_to_waypoint(14)

                    # moving back to the dispenser
                    self.move_to_waypoint(10)
                    self.move_to_waypoint(9)
                    self.move_to_waypoint(8)
                    self.move_to_waypoint(7)
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(1)

                elif (table_num == 3):
                    # moving to the table
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(7)

                    # moving back to the dispenser
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(1)

                elif (table_num == 4):
                    # moving to the table
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(3)
                    self.move_to_waypoint(6)

                    # moving back to the dispenser
                    self.move_to_waypoint(3)
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(1)

                elif (table_num == 5):
                    # moving to the table
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(3)
                    self.move_to_waypoint(4)
                    self.move_to_waypoint(5)

                    # moving back to the dispenser
                    self.move_to_waypoint(4)
                    self.move_to_waypoint(3)
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(1)
                
                else: # table 6
                    # moving to the table
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(7)
                    self.move_to_waypoint(8)
                    self.move_to_waypoint(9)
                    self.move_to_waypoint(10)
                    self.move_to_waypoint(11)
                    self.move_to_waypoint(12)
                    self.move_to_waypoint(13)

                    # moving back to the dispenser
                    self.move_to_waypoint(12)
                    self.move_to_waypoint(11)
                    self.move_to_waypoint(10)
                    self.move_to_waypoint(9)
                    self.move_to_waypoint(8)
                    self.move_to_waypoint(7)
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(1)
        
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