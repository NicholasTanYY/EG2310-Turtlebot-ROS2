# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# adapted from https://github.com/Shashika007/teleop_twist_keyboard_ros2/blob/foxy/teleop_twist_keyboard_trio/teleop_keyboard.py

import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import numpy as np
import cmath

# constants
rotatechange = 0.1
speedchange = 0.05

arr=[]
num_waypoints, entries = (4, 3)
for i in range(num_waypoints):
    col = []
    for j in range(entries):
        col.append(0)
    arr.append(col)

f_path = '/home/nicholas/colcon_ws/src/auto_nav/auto_nav/waypoint_log.txt'

class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist,'cmd_vel',10)

        self.odom_subscriber = self.create_subscription(
                Odometry, 
                'odom', 
                self.odom_callback, 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.odom_subscriber
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.yaw = 0.0
        
    def odom_callback(self, msg):
        
        orientation_quat = msg.pose.pose.orientation
        quaternion = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion(quaternion)
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

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
    
# function to read keyboard input
    def readKey(self):
        twist = geometry_msgs.msg.Twist()
        try:
            waypoint = 0
            while waypoint < num_waypoints:
                # get keyboard input
                print("Press p to set waypoint")
                rclpy.spin_once(self)
                cmd_char = str(input("Keys w/x a/d s: "))
        
                # check which key was entered
                if cmd_char == 's':
                    # stop moving
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif cmd_char == 'w':
                    # move forward
                    twist.linear.x += speedchange
                    twist.angular.z = 0.0
                elif cmd_char == 'x':
                    # move backward
                    twist.linear.x -= speedchange
                    twist.angular.z = 0.0
                elif cmd_char == 'a':
                    # turn counter-clockwise
                    twist.linear.x = 0.0
                    twist.angular.z += rotatechange
                elif cmd_char == 'd':
                    # turn clockwise
                    twist.linear.x = 0.0
                    twist.angular.z -= rotatechange
                elif cmd_char == 'p':
                    # set the current point as a waypoint. Get the x and y coordinates as well as the value for yaw
                    # store in the format [x, y, yaw]
                    
                    arr[waypoint][0] = self.odom_x
                    arr[waypoint][1] = self.odom_y
                    arr[waypoint][2] = self.yaw                 

                    self.get_logger().info('Waypoint logged!')
                    print(arr)
                    waypoint += 1

                # start the movement
                self.publisher_.publish(twist)
                
                
        except Exception as e:
            print(e)
            
		# Ctrl-c detected
        finally:
            # stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

            # write to waypoint file
            waypoint_arr = np.array(arr)
            np.savetxt(f_path, waypoint_arr)



def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    mover.readKey()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mover.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
