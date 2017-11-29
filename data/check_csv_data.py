#!/usr/bin/python

import os 
import csv
#Go through all the data files and suggest validation/normalization stuff

#Get all csv files in the directory
files = [f for f in os.listdir('.') if os.path.isfile(f) and f.endswith(".csv")]
for f in files:
	with open(f,"r") as infile:
		csvFile = csv.DictReader(infile,delimiter=',')
		for line in csvFile:
			#Check that each field has a value, and that all the values are there
			for key in line.keys():
				if line[key] is None:
					print "p{0}.csv timestamp {1} has None for {2}".format(line["user"], line["time"], key)

#For each file, read the file

#Check that each line has the same number of commmas

#Check that timestamps are monotonically increasing

#list all unique codes

