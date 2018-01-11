#!/usr/bin/python

# count all the events in a json file 
# To call on every file in a directory, you can do this:
# for file in `ls *.json`; do outfile=$(basename "$file"); outfile="${outfile%.*}"; echo $file > $outfile.txt; ./count_events.py $file >> $outfile.txt; done
# If you get compaints from the json loader, use:  sed -i 's/\x0//g' *.json
# to strip null bytes from the json files. 

import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("inFile", nargs = 1, help="File to print")
args = parser.parse_args()

#Get the file name
fname = args.inFile[0]

class counter(object):
	def __init__(self):
		self.counts = {}

	def add(self, item):
		if item in self.counts.keys():
			#We already have a counter for these, increment it
			self.counts[item] += 1
		else:
			#This is the first of this item we've seen
			self.counts[item] = 1

	def get_counts(self):
		return self.counts

	def print_counts(self):
		for item in self.counts.keys():
			print "{0}:\t{1}".format(item, self.counts[item])

tasks = {}

with open(fname, "r") as infile:
	data = json.loads(infile.read())
	for task in sorted(data['tasks'].keys(), key=int):
		tc = counter()
		events = data['tasks'][task]
		#Only care about events with times
		events = [x for x in events if 'time' in x.keys()]
		for event in sorted(events, key=lambda x: float(x['time'])):
			tc.add(event['event_type'])
		tasks[int(task)] = tc

#Print the task counts as a csv file
header = "particpant,gesture,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18"
line = fname + ","
print header
for gesture in ['pinch','drag','other','tap','voice_command','lasso','ui','box_select']:
	gesture_line = line + gesture + ","
	for task in range(1,18):
		if task in tasks.keys():
			gestures = tasks[task].get_counts()
			if gesture in gestures.keys():
				#The user made this gesture in this task
				gesture_line += str(gestures[gesture]) + ","
			else:
				#The user did not make this gesture in the task
				gesture_line += ","
		else:
			#The user didn't do this task
			gesture_line += ","
	print gesture_line
