#!/bin/bash
# place this file in ~/colcon_ws and run it from that directory

gnome-terminal --tab --title="map2base (keep running)" --command="ros2 run auto_nav map2base_node"
gnome-terminal --tab --title="set_waypoints" --command="ros2 run auto_nav set_waypoints"
gnome-terminal --tab --title="rteleop" --command="ros2 run turtlebot3_teleop teleop_keyboard"
