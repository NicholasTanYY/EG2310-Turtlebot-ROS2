#!/usr/bin/env python3
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

class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist,'cmd_vel',10)

    def readKey(self):
        twist = geometry_msgs.msg.Twist()
        try:
            while True:
                # get keyboard input
                cmd_char = str(input("Keys w/x a/d s: "))
        
                # check which key was entered
                if cmd_char == 's':
                    # stop moving
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif cmd_char == 'w':
                    # move forward
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0
                elif cmd_char == 'x':
                    # move backward
                    twist.linear.x = -0.1
                    twist.angular.z = 0.0
                elif cmd_char == 'a':
                    # turn counter-clockwise
                    twist.linear.x = 0.0
                    twist.angular.z = 1.0
                elif cmd_char == 'd':
                    # turn clockwise
                    twist.linear.x = 0.0
                    twist.angular.z = -1.0

                # log the info
                self.get_logger().info(cmd_char)
                self.publisher_.publish(twist)
                
        except Exception as e:
            print(e)
            
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    mover.readKey()
    rclpy.spin(mover)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
