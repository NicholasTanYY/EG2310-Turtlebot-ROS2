import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool
from custom_msgs.msg import Button
import math
import numpy as np
import time

box_thres = 0.15
rotate_change = 0.35
slow_rotate_change = 0.10
speed_change= 0.10
time_threshold = 1.28 * box_thres / speed_change
dist_threshold = 0.18        # Distance threshold for the robot to stop in front of the pail
initial_yaw = 0.0
front_angle = 3
front_angle_6 = 65
front_angle_range = range(-front_angle,front_angle+1,1)
stop_distance = 0.25
occ_bins = [-1, 0, 50, 101]
# f_path = '/home/nicholas/colcon_ws/src/auto_nav/auto_nav/waypoint_logging/waypoint_log.txt'
f_path = '/home/nicholas/colcon_ws/src/auto_nav/auto_nav/waypoint_logging/actual_waypoints.txt'
# f_path = '/home/nicholas/colcon_ws/src/auto_nav/auto_nav/waypoint_logging/test_waypoints.txt'

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

    # print("target_yaw = ", target_yaw)
    # print("current_yaw = ", current_yaw)
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
            qos_profile=qos_profile_sensor_data)

        self.mqtt_subscription = self.create_subscription(
            String,
            'mqtt_data',
            self.mqtt_callback,
            1)
        
        self.button_subscription = self.create_subscription(
            Button,
            'button_pressed',
            self.button_callback,
            10)
        self.buttonpressed = False

        self.node = rclpy.create_node('button_subscriber')
        self.node.create_subscription(Bool, 'button_pressed', self.button_callback, 10)

        self.cmd = Twist()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.laser_range = np.zeros((2*front_angle+1,))
        self.laser_range_6 = np.zeros((2*front_angle_6+1,))
        self.laser_forward = 0.0
        self.mapbase = Pose().position
        self.x_coordinate = 0
        self.y_coordinate = 0
        self.storeX, self.storeY = 0, 0
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
        self.x_coordinate = 0.0
        self.y_coordinate = 0.0
        self.table6_turn_angle = 0
        self.laser_angle_increment = 0
    
    def Clocking(self):
        time_started = self.get_clock().now().to_msg().sec
        time_elapsed = 0
        while time_elapsed <= time_threshold:
            rclpy.spin_once(self)
            time_diff = self.get_clock().now().to_msg().sec - time_started
            time_elapsed = self.get_clock().now().to_msg().nanosec/float(1000000000) + time_diff
            # print(self.cmd.linear.x)
    
    def timer_callback(self):
        pass

    def map2base_callback(self, msg):
        
        orientation_quat = msg.orientation
        quaternion = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)
        self.mapbase = msg.position
        self.x_coordinate = msg.position.x
        self.y_coordinate = msg.position.y

    def laser_callback(self, msg):

        # rclpy.spin_once(self)
        self.laser_range = np.array(msg.ranges[0:front_angle] + msg.ranges[-front_angle:])
        self.laser_range_6 = np.array(msg.ranges[0:front_angle_6] + msg.ranges[-front_angle_6:])
        self.laser_range[self.laser_range==0] = np.nan
        self.laser_range_6[self.laser_range_6==0] = np.nan
        self.laser_angle_increment = msg.angle_increment
        self.laser_forward = np.nanmean(self.laser_range) / 2
        self.laser_forward_6 = np.nanmean(self.laser_range_6) / 2

        # print("Laser range = ", self.laser_range)
        # print("Laser range 6 = ", self.laser_range_6)

    def mqtt_callback(self, msg):
        data = msg.data
        print("Received MQTT data:", data)
        if data == "NIL":
            self.mqtt_val = 0
            print("MQTT data is NIL")
        else:
            self.mqtt_val = int(data)

    def button_callback(self, msg):
        if msg.data:
            # print("Button pressed!")
            # self.get_logger().info('In button_callback')
            self.buttonpressed = True
        else:
            # print("Button released")
            self.buttonpressed = False
        
    def stopbot(self, delay):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        time.sleep(delay)

    
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

        # self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        self.cmd.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(self.cmd)

    def MoveForward(self, x_coord, y_coord):
        
        self.get_logger().info('Moving Forward...')
        while not ((self.mapbase.y < y_coord + box_thres and self.mapbase.y > y_coord - box_thres) 
                   and (self.mapbase.x < x_coord + box_thres and self.mapbase.x > x_coord - box_thres)):
            rclpy.spin_once(self)
            # self.get_logger().info('I receive "%s"' % str(self.mapbase.y))
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)

        self.Clocking()
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)

    def Reverse(self, x_coord, y_coord):
        
        self.get_logger().info('Reversing...')
        while not ((self.mapbase.y < y_coord + box_thres and self.mapbase.y > y_coord - box_thres) 
                   and (self.mapbase.x < x_coord + box_thres and self.mapbase.x > x_coord - box_thres)):
            rclpy.spin_once(self)
            # self.get_logger().info('I receive "%s"' % str(self.mapbase.y))
            self.cmd.linear.x = -speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
        
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)
    
    def move_to_waypoint(self, WP_num):
        x1, y1, x2, y2, current_yaw = self.x_coordinate, self.y_coordinate, 
        self.waypoint_arr[WP_num][0], self.waypoint_arr[WP_num][1], self.yaw
        yaw_difference, distance = calculate_yaw_and_distance(x1, y1, x2, y2, current_yaw)
        yaw_difference = yaw_difference / math.pi * 180

        self.get_logger().info('Moving to waypoint %d' % WP_num)

        self.rotatebot(yaw_difference)
        self.stopbot(0.1)

        self.MoveForward(x2, y2)
        self.stopbot(0.1)

        self.get_logger().info('Waypoint reached!')

    def reverse_to_waypoint1(self):

        x2, y2 = self.waypoint_arr[1][0], self.waypoint_arr[1][1]
        self.get_logger().info('Moving to waypoint 1')
        self.Reverse(x2, y2)
        self.stopbot(0.1)
        self.get_logger().info('Waypoint 1 reached!')

    def move_close(self):
        # scan the front of the robot to check the distance to the pail
        print("Moving closer to the pail...")
        print(self.laser_forward)
        while self.laser_forward > dist_threshold:
            rclpy.spin_once(self)
            # print("Moving ...")
            self.cmd.linear.x = speed_change
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
        
        self.cmd.linear.x = 0.0
        self.publisher_.publish(self.cmd)
    
    def wait_for_button_press(self):
        print("Waiting for button press...")
        while not self.buttonpressed:
            rclpy.spin_once(self.node)

    def wait_for_button_release(self):
        print("Waiting for button release")
        while self.buttonpressed:
            rclpy.spin_once(self.node)
        print("Button released.")

    def move_to_table6(self):
        # turn to that angle      
        self.rotatebot(math.degrees(-self.yaw))
        rclpy.spin_once(self)

        # print("Laser range = ", self.laser_range)
        # print("Laser range 6 = ", self.laser_range_6)

        # Calculate angle of minimum range value
        min_range_index = np.nanargmin(self.laser_range_6)
        min_range = self.laser_range_6[min_range_index]
        min_range_angle = (min_range_index - front_angle_6) * self.laser_angle_increment

        # Convert angle to degrees and print result
        if min_range_angle < 0:
            min_range_angle_degrees = -(front_angle_6 + math.degrees(min_range_angle))
        else:
            min_range_angle_degrees = front_angle_6 - math.degrees(min_range_angle)
        
        # print("Angle to turn: ", min_range_angle_degrees)
        self.table6_turn_angle = min_range_angle_degrees

        print("Turning to table 6...")
        print("Angle to turn: ", self.table6_turn_angle)
        self.rotatebot(-self.table6_turn_angle)

        self.move_close()

    def dock(self):
        self.move_to_waypoint(1)
        self.move_to_waypoint(18)
        self.move_to_waypoint(0)
        self.rotatebot(-self.yaw)

    def face_front(self):
        rotation_angle = ((-(math.pi / 2) ) - self.yaw)
        self.rotatebot(math.degrees(rotation_angle))

    def motion(self):
        
        try:
            while rclpy.ok():

                print("current yaw = ", self.yaw)
                print("current Xpos = ", self.x_coordinate)
                print("current Ypos = ", self.y_coordinate)
                
                self.wait_for_button_press()

                print("Waiting for mqtt input...")
                while self.mqtt_val == 0:
                    rclpy.spin_once(self)
                
                table_num = self.mqtt_val
                print("table_num received = ", table_num)
                self.reverse_to_waypoint1()

                if (table_num == 1):
                    # moving to the table
                    self.move_to_waypoint(9)
                    self.move_to_waypoint(16)
                    self.face_front()
                    self.move_close()

                    self.wait_for_button_release()

                    # moving back to the dispenser
                    self.move_to_waypoint(9)
                    self.dock()

                elif (table_num == 2):
                    # moving to the table
                    self.move_to_waypoint(9)
                    self.move_to_waypoint(10)
                    self.move_to_waypoint(15)
                    self.face_front()
                    self.move_close()

                    self.wait_for_button_release()

                    # moving back to the dispenser
                    self.move_to_waypoint(9)
                    self.dock()

                elif (table_num == 3):
                    # moving to the table
                    self.move_to_waypoint(7)
                    self.face_front()
                    self.move_close()

                    self.wait_for_button_release()

                    # moving back to the dispenser
                    self.dock()

                elif (table_num == 4):
                    # moving to the table
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(6)
                    self.face_front()
                    self.move_close()

                    self.wait_for_button_release()

                    # moving back to the dispenser
                    self.move_to_waypoint(2)
                    self.dock()

                elif (table_num == 5):
                    # moving to the table
                    self.move_to_waypoint(2)
                    self.move_to_waypoint(3)
                    self.move_to_waypoint(4)
                    self.move_to_waypoint(5)
                    self.face_front()
                    self.move_close()

                    self.wait_for_button_release() 

                    # moving back to the dispenser
                    self.move_to_waypoint(4)
                    self.move_to_waypoint(3)
                    self.move_to_waypoint(2)
                    self.dock()
                
                else: # table 6
                    # moving to the table
                    self.move_to_waypoint(17)
                    self.move_to_waypoint(12)
                    self.move_to_waypoint(13)
                    self.move_to_waypoint(14)
                    self.move_to_table6()

                    self.wait_for_button_release()

                    # moving back to the dispenser
                    self.move_to_waypoint(14)
                    self.move_to_waypoint(13)
                    self.move_to_waypoint(12)
                    self.move_to_waypoint(17)
                    self.dock()
                
                self.buttonpressed = False  # reset the buttonpressed to False
                self.mqtt_val = 0       # reset the mqtt_val to 0
        
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
