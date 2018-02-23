#!/usr/bin/python

"""
Script for reading coding scripts and converting them to json with code_video.py
"""

from code_video import VideoCodeCmd
import sys

if __name__== '__main__':
    vd = VideoCodeCmd()
    if len(sys.argv) < 2:
        print("Please provide a file path")
    print sys.argv[1]
    for path in sys.argv[1:]:
        input_file = open(path)
        for line in input_file:
            vd.onecmd(line)

    vd.onecmd("quit")
