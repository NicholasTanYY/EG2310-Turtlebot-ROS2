#!/bin/bash
# place this file in ~/colcon_ws and run it from that directory

gnome-terminal --tab --title="map2base (keep running)" --command="ros2 run auto_nav map2base_node"
gnome-terminal --tab --title="receiver/topic sender" --command="python3 src/auto_nav/auto_nav/mqtt/receiver.py"
gnome-terminal --tab --title="sim_navi" --command="ros2 run auto_nav sim_navi"
