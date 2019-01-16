#!/usr/bin/python

#Generates spoof april tags based on the command line 

import argparse
import rospy
from apriltags_ros.msg import *
import cv2

parser = argparse.ArgumentParser(description='Publish AprilTag messages based on the command line rather than actual tags.')
parser.add_argument('--img', dest='img_path', action='store', default="",
                    help='path to image file to use')
parser.add_argument('--rate', dest='pub_rate', action='store_const', default=10,
                    help='rate setting, default is 10Hz')


