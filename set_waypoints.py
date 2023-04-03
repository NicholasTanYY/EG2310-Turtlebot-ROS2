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
from geometry_msgs.msg import Pose
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

num_waypoints = 4

entries = 3
arr=[]
for i in range(num_waypoints):
    col = []
    for j in range(entries):
        col.append(0)
    arr.append(col)

f_path = '/home/nicholas/colcon_ws/src/auto_nav/auto_nav/waypoint_log.txt'

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

class Mover(Node):
    def __init__(self):
        super().__init__('mover')

        self.map_frame_subscriber = self.create_subscription(
                Pose, 
                'map2base',
                self.map_callback, 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.map_frame_subscriber

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        
    def map_callback(self, msg):
        
        orientation_quat = msg.orientation
        quaternion = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)
        self.pos_x = msg.position.x
        self.pos_y = msg.position.y
    
# function to read keyboard input
    def readKey(self):
        try:
            print("Run map2base publisher along with this.")
            print("Run teleop_keyboard along with this using the command rteleop.")
            print("\nPress p once before setting waypoints\n")

            waypoint = 0
            while waypoint < num_waypoints:
                # get keyboard input
                
                rclpy.spin_once(self)
                cmd_char = str(input("Press p to set waypoint: "))
                    
                # check which key was entered
                if cmd_char == 'p':
                    # set the current point as a waypoint. Get the x and y coordinates as well as the value for yaw
                    # store in the format [x, y, yaw]

                    rclpy.spin_once(self)
                    arr[waypoint][0] = self.pos_x
                    arr[waypoint][1] = self.pos_y
                    arr[waypoint][2] = self.yaw            

                    self.get_logger().info(f'Waypoint {waypoint} logged!')
                    print(arr[waypoint])
                    # print(arr[1:])

                    # if waypoint == 0:
                    #     print("Ignore values above. Start plotting waypoints now.")
                    waypoint += 1
                
                
        except Exception as e:
            print(e)
            

		# Ctrl-c detected
        finally:
            # write to waypoint file
            # new_arr = arr[1:]        # debugging error for identical first 2 entries
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
