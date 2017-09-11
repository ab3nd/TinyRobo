#!/usr/bin/python

# Generate the UI test ini file
import os

sections = {"files_single":"./1","files_10":"./10","files_100":"./100","files_100":"./1000","files_unknown":"./unknown"}

for section in sections.keys():
	print "[{0}]".format(section)
	fileList = sorted(os.listdir(sections[section]))
	for index, fileName in enumerate(fileList):
		print "{0} = {1}/{2}".format(index + 1, sections[section], fileName)
	print " "