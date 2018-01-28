#!/usr/bin/python

#Compare two .json files coding for the same events, calculate Krippendorff's alpha for them
import json
import argparse
import sys
import os

def load_file(fname):
	with open(fname, "r") as infile:
		data = json.loads(infile.read())
		return data


class event_counter(object):
	def __init__(self):
		self.counts = {}
		self.total_events = 0

	def add_data(self, data):
		#Get the tasks
		tasks = sorted(data['tasks'].keys(), key=int) 		

		for task in tasks:

			#Discard memos
			events = [x for x in data['tasks'][task] if x['event_type'] != 'memo']
			
			#Count all events by type
			for event in events:
				self.total_events += 1
				if event['event_type'] in self.counts.keys():
					self.counts[event['event_type']] += 1
				else:
					self.counts[event['event_type']] = 1

	def print_counts(self):
		#Now print out the counts and the prevelence
		for event_type in self.counts.keys():
			print event_type, self.counts[event_type], (float(self.counts[event_type])/self.total_events)*100,"%"


# #The only two arguments are the file names
# parser = argparse.ArgumentParser()
# parser.add_argument("inFile", nargs = 1, help="JSON file to calculate prevelences")
# args = parser.parse_args()

def get_files(endswith = ".json"):
	jsonfiles = [x for x in os.listdir("./") if x.endswith(endswith)]
	return jsonfiles

ec = event_counter()

for file in get_files():	
	#Load the file up
	data = load_file(file)

	ec.add_data(data)

ec.print_counts()

