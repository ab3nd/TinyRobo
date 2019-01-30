#!/usr/bin/python

#Go through all the bag files in user and task order, and print out the gestures detected

#Get a list of all the files, sort by user and task

import os
import re
import fnmatch
import rosbag
import rospy
import yaml
import json


def get_dir_list(path, pattern):
    dirs = [d for d in os.listdir(path) if os.path.isdir(os.path.join(path, d))]
    filterDirs = [d for d in dirs if re.match(pattern, d)]
    return filterDirs

#From https://stackoverflow.com/questions/1724693/find-a-file-in-python
def find(pattern, path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result

#Load the coding data
fname = "/home/ams/TinyRoboData/all_participants.json"
data_file = open(fname, 'r')
data = json.loads(data_file.read())

for directory in sorted(get_dir_list("./good_run/", "^u[0-9]+_c*")):

    #get the user, condition, and task from the directory name
    user, cond, task = directory.split('_')
    user = user.strip("u")
    cond = cond.strip("c")
    task = task.strip("t")
    #get the bagfile
    bagname = find("{}_gestures.bag".format(directory), "./good_run/{}".format(directory))[0]
    bag = rosbag.Bag(bagname)
    bagInfo = yaml.load(bag._get_yaml_info())
    if 'start' in bagInfo.keys():
        startTime = bagInfo['start']
    else:
        #This means the bag has no recognized gestures
        print bagname, ",,,"
        print "-- No recognized gestures --,,,"
        print ''
        continue


    detected_gestures = []
    for topic, msg, t in bag.read_messages():
        if topic == "/gestures":
            #Caclulate how far into the task the event occured (as oppsed to wall time)
            #Wall time doesn't mean much, as the gestures were generated in late January
            #from data recorded in October of years earlier. 
            timeOffset = t.to_sec() - startTime
            detected_gestures.append((msg.eventName, "{0:.4f}".format(timeOffset)))

    # print bagname
    # print user, task
    # print ''
    coded_gestures = []
    gestures_from_coding = data[user]['tasks'][task]
    for gesture in gestures_from_coding:
        if gesture['event_type'] == 'memo': #Don't print memo events
            continue
        else:
            if not gesture['example']: #Don't print gestures intended as examples
                coded_gestures.append((gesture['event_type'], "{0:0.4f}".format(gesture['time'])))
    
    #Build a pad and stick it on whichever list is shorter
    pad = [(None, 0.0)] * abs(len(coded_gestures) - len(detected_gestures))
    if len(coded_gestures) < len(detected_gestures):
        coded_gestures = coded_gestures + pad
    elif len(coded_gestures) > len(detected_gestures):
        detected_gestures = detected_gestures + pad

    # print bagname
    # print "{0:<23}{1}".format("detected", "coded")
    # for g1, g2 in zip(detected_gestures, coded_gestures):
    #     print "{0: <14}{1: <9}{2: <14}{3}".format(g1[0], g1[1], g2[0], g2[1])
    # print ""

    print bagname,",,,"
    print "{0},,{1},".format("detected", "coded")
    for g1, g2 in zip(detected_gestures, coded_gestures):
        print "{0},{1},{2},{3}".format(g1[0], g1[1], g2[0], g2[1])
    print ""