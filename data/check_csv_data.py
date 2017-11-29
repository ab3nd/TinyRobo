#!/usr/bin/python

import os 
import csv
#Go through all the data files and suggest validation/normalization stuff


#Store counts of all of the codes
all_codes={}

def add_code(code):
	if code in all_codes.keys():
		all_codes[code] += 1
	else:
		all_codes[code] = 1

#Get all csv files in the directory
files = [f for f in os.listdir('.') if os.path.isfile(f) and f.endswith(".csv")]
for f in files:
	with open(f,"r") as infile:
		csvFile = csv.DictReader(infile,delimiter=',')

		#Used to check if timestamps are monotonically increasing
		lastLineStamp = None

		for line in csvFile:
			#Check that each field has a value, and that all the values are there
			for key in line.keys():
				if line[key] is None:
					print "p{0}.csv timestamp {1} has None for {2}".format(line["user"], line["time"], key)
			
			#Add the code to the count
			add_code(line["event"])

			#Check that timestamps are monotonically increasing
			if lastLineStamp is None:
				lastLineStamp = float(line["time"])
			else:
				currentStamp = float(line["time"])
				if lastLineStamp < currentStamp:
					#It's legit, time has advanced
					lastLineStamp = currentStamp
				else:
					#It's not legit, time has flowed in reverse
					print "p{0}.csv timestamp {1} exhibits retrochronality".format(line["user"], line["time"])

#list all unique codes

