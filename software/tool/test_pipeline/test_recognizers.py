#!/usr/bin/python

# Coordinates test runs of the user gesture pipeline

import roslaunch
import rospy
import argparse
import subprocess
import os
import fnmatch

#Map of tasks to task numbers
# task 								1	10	100	1000	Unknown    task ID number
#---------------------------------------------------------------------------------
# Move to A							1	1	1	1		1			1
# Move to A with wall				2	2	2	2		2			2
# Stop the robots					3	3	3	3		3			3
# Divide around obstacle				4	4	4		4			4
# Orange to B, red to A				4	5	5	5		5			5
# Orange to A, red to B				5	6	6	6		6			6
# Orange to A, red to B (mixed)			7	7	7		7			7
# Divide group						6	8	8	8		8			8
# Merge group							9	9	9		9			9
# Form a line							10	10	10		10			10
# Form a square							11	11	11		11			11
# Move crate to A					7	12	12	12		12			12
# Move crate to A (dispersed)			13	13	13		13			13
# Mark defective					8	14	13	14					14
# Remove defective					9	15	14	15					15
# Patrol screen						10	16	16	16		14			16
# Patrol A							11	17	17	17		15			17
# Disperse								18	18	18		16			18

p_to_cond = {0:0, 1:1, 2:10, 3:100, 4:1000}

#Condition and task number in sequence to task ID number
#A task has the same ID across conditions, but not all 
#conditions have the same set of task IDs
task_and_cond_to_ID = {
0:{1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,16:14,17:15,16:18},
1:{1:1,2:2,3:3,5:4,6:5,8:6,12:7,14:8,15:9,16:10,17:11},
10:{1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18},
100:{1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18},
1000:{1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18}
}

#Condition and task to slide number
task_and_cond_to_slide = {
0:{1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,16:14,17:15,18:16},
1:{1:1,2:2,3:3,5:4,6:5,8:6,12:7,14:8,15:9,16:10,17:11},
10:{1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18},
100:{1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18},
1000:{1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18}	
}

def getSlideFile(taskID, condition):
	fname = None
	slide = None
	if condition == 'X':
		slide = (task_and_cond_to_slide[0][int(taskID)] * 2) + 1
		fname = "/home/ams/TinyRobo/software/catkin_ws/src/user_interface/src/unknown/Swarm_Robot_Control_-_Unknown_Number_of_Robots_{0:04d}.png".format(slide)	
	elif condition == '1':
		slide = (task_and_cond_to_slide[0][int(taskID)] * 2) + 1
		fname = "/home/ams/TinyRobo/software/catkin_ws/src/user_interface/src/{0}/Swarm_Robot_Control_-_Single_Robot_{1:04d}.png".format(condition, slide)
	else:
		#Compute slide number 
		slide = (int(taskID) * 2) + 1
		fname = "/home/ams/TinyRobo/software/catkin_ws/src/user_interface/src/{0}/Swarm_Robot_Control_-_{0}_Robot_{1:04d}.png".format(condition, slide)
	if fname is not None:
		return fname
	else:
		raise ValueError("Couldn't get a slide file name for condition {0}, task {1}".format(condition, task))
	

#From https://stackoverflow.com/questions/1724693/find-a-file-in-python
def find(pattern, path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result

def get_bagfile_path():
	path = "/home/ams/.ros"
	pattern = "recognizer_test*"

	#Get the files
	files = find(pattern, path)

	#Because the file names contains dates, this should more or less get the oldest one
	return sorted(files)[0]


# Start up all the nodes
# Houston is launch control, lol
rospy.init_node('houston', anonymous=True)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
#This includes a rosbag record to get the results of the recognizers
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ams/TinyRobo/software/catkin_ws/src/tiny_robo_launch/launch/test_recognizers.launch"])
launch.start()
rospy.loginfo("Started all the pipeline nodes")

#Get the command line argument
parser = argparse.ArgumentParser(description="Run a bagfile of touches through the recognizers")
parser.add_argument('bagfileName', type=str, nargs=1, help='path to the bagfile')
args = parser.parse_args()

#Build the path to the image to use from the command line argument bag file
inputFile = args.bagfileName[0]
chunks = inputFile.split("-")
user = chunks[1].split("_")[0]
condition = chunks[2].split("_")[0]
task = chunks[3].split("_")[0]

#Get the slide number from the task number
#If I was clever I guess I would have put that instead of the task number in the file name...
spoofFile = getSlideFile(task, condition)

#invoke the spoof april tag recognizer 
rospy.loginfo("Spoofing AprilTag pixel locations with {}".format(spoofFile))
spooftagsP = subprocess.Popen(["python", "/home/ams/TinyRobo/software/catkin_ws/src/spoof_apriltags/src/spoof_pixels.py", spoofFile])


# wait briefly while they come on line
rospy.sleep(3)

#play the gesture bag file (passed on the command line)
rosbagP = subprocess.Popen(["rosbag", "play", inputFile])
rospy.loginfo("Playing bag, please wait")

#Wait for the rosbag to finish playing
rosbagP.wait()
rospy.loginfo("Bag over, packing up")

#Wait a couple of seconds for any last messages to arrive
rospy.sleep(2)

#Shut everything down
spooftagsP.terminate()
launch.shutdown()

#Get the resulting bagfile
bag = get_bagfile_path()

#Move it to an appropriately named directory
os.renames(bag, "u{0}_c{1}_t{2}/u{0}_c{1}_t{2}_gestures.bag".format(user,condition,task))