#!/usr/bin/python

# Coordinates test runs of the user gesture pipeline

import roslaunch
import rospy
import argparse

# Start up all the nodes
# Houston is launch control, lol
# rospy.init_node('houston', anonymous=True)

# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
# #This includes a rosbag record to get the results of the recognizers
# launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ams/TinyRobo/software/catkin_ws/src/tiny_robo_launch/launch/test_recognizers.launch"])
# launch.start()
# rospy.loginfo("started all the pipeline nodes")

#TODO include an arg to the launchfile to set which image the spoof april tag recognizer uses

#Get the command line argument
parser = argparse.ArgumentParser(description="Run a bagfile of touches through the recognizers")
parser.add_argument('bagfileName', type=str, nargs=1, help='path to the bagfile')
args = parser.parse_args()

#Build the path to the image to use from that

inputFile = args.bagfileName[0]
chunks = inputFile.split("-")
condition = chunks[2].split("_")[0]
task = chunks[3].split("_")[0]

fname = "/home/ams/TinyRobo/software/catkin_ws/src/user_interface/src/{0}/Swarm_Robot_Control_-_{0}_Robot_0000.png".format(condition)

import pdb; pdb.set_trace()

#invoke the spoof april tag recognizer 

#TODO wait briefly while they come on line

#TODO play the gesture bag file (passed on the command line)


#Delay forever
rospy.spin()
launch.shutdown()

