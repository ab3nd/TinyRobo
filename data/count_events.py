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


c = counter()

with open(fname, "r") as infile:
	data = json.loads(infile.read())
	for task in sorted(data['tasks'].keys(), key=int):
		events = data['tasks'][task]
		#Only care about events with times
		events = [x for x in events if 'time' in x.keys()]
		for event in sorted(events, key=lambda x: float(x['time'])):
			c.add(event['event_type'])

c.print_counts()