#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#echo "This is an empty launch script. Update it to launch your application."
roslaunch my_package proj_goto_1.launch veh:=$VEHICLE_NAME
#roslaunch my_package_2 localization_node_test.launch veh:=$VEHICLE_NAME
#rosrun my_package my_node.py
