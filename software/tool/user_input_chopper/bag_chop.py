#!/usr/bin/python

# Given a bag file, chop it into the gestures the participant used for each task

import rosbag
import rospy
import yaml
import argparse
import os.path

parser = argparse.ArgumentParser(description="Convert a bagfile from Abe's experiment into per-task inputs")
parser.add_argument('bagfileName', type=str, nargs=1, help='path to the bagfile')
args = parser.parse_args()

#Get the file name and load the bagfile
name = args.bagfileName[0]
bag = rosbag.Bag(name)

#Bagfiles were saved with names of the format: id_15_cond_1000_2017-10-20-12-45-27.bag
#Split on _, get out id and condtion
as_list = os.path.basename(name).split("_")
participant = as_list[1]
condition = as_list[3]

#Get info about the bag
bagInfo = yaml.load(bag._get_yaml_info())

for topic in bagInfo['topics']:
	print "{0} \t{1} \tfrequency:{2} \tmsg count:{3}".format(topic['topic'], topic['type'], topic['frequency'], topic['messages'])

#1,10,100,1000 robot conditions, screens the user interacts with start at screen 4 for task one
scrOffset = 4
if condition == 'X':
	#Unknown robot condition, screens the user interacts with start at screen 6 for task 1
	scrOffset = 6
	
screenCounter = 0
touchCounter = 0
taskCounter = 0

outBag = None

for topic, msg, t in bag.read_messages():
	#There are several other message types, but these are the ones we care about
	if topic == "/ui_image":
		#New UI screen
		screenCounter += 1
		if screenCounter % 2 == 0 and screenCounter >= scrOffset:
			if taskCounter > 0:
				#Not the first task, so close the file
				outBag.close()
			taskCounter += 1
			outBag = rosbag.Bag('./user-{}_cond-{}_task-{}.bag'.format(participant, condition, taskCounter), 'w')
			print "Task {} got {} touches".format(taskCounter, touchCounter)
		touchCounter = 0
	if topic == "/touches":
		touchCounter += 1
		if outBag is not None:
			outBag.write(topic, msg, t)
