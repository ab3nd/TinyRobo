#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file. This script also launches Argos. 

ARGOS_CFG=/home/ams/TinyRobo/software/catkin_ws/src/argos_ros_bridge/argos_worlds/live_ui_arena.argos
argos3 -c $ARGOS_CFG &

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
n=6

LAUNCH_FILE=/tmp/argos_bridge.launch

echo "<launch>" > $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="bot$i"
    echo -e "\t<group ns=\"$namespace\">"
    echo -e "\t\t<node pkg=\"gcpr_runner\" type=\"argos_gcpr.py\" name=\"bot_controller\" output=\"screen\" />"
    echo -e "\t</group>"
done >> $LAUNCH_FILE
echo -e "</launch>" >> $LAUNCH_FILE

roslaunch $LAUNCH_FILE
