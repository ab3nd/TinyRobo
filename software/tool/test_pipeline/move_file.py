#!/usr/bin/python

#Get a file that starts with "recognizer_test" in the ~/.ros/ directory, and move it to a new directory

import json
import rosbag
import rospy
import os
import fnmatch
import yaml

#From https://stackoverflow.com/questions/1724693/find-a-file-in-python
def find(pattern, path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result

path = "/home/ams/.ros"
pattern = "recognizer_test*"

#Get the files
files = find(pattern, path)

#Because the file names contains dates, this should more or less get the oldest one
oldName = sorted(files)[0]

#Move it to an appropriately named directory
os.renames(oldName, "test_{0}/{0}_{0}.bag".format('foo'))