#!/usr/bin/python

#Prettyprint json files
# To call on every file in a directory, you can do this:
# for file in `ls *.json`; do outfile=$(basename "$file"); outfile="${outfile%.*}"; echo $file > $outfile.txt; ./ppjson.py $file >> $outfile.txt; done

import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("inFile", nargs = 1, help="File to print")
args = parser.parse_args()

#Get the file name
fname = args.inFile[0]

#Load that and get the data
with open(fname, "r") as infile:
	data = json.loads(infile.read())
	for task in sorted(data['tasks'].keys(), key=int):
		print task
		events = data['tasks'][task]
		#Only care about events with times
		events = [x for x in events if 'time' in x.keys()]
		for event in sorted(events, key=lambda x: float(x['time'])):
			desc_str = ""
			#Add the object
			if 'objects' in event.keys():
				desc_str += (" ".join(event['objects'])).encode("utf-8")
			#If there's a description, add that
			if 'description' in event.keys():
				desc_str +=  " (" + (" ".join(event['description'])).encode("utf-8") + ")"
			#print the result with nice(ish) tabulation
			print "\t{0}\t{1} {2}".format(event['time'], event['event_type'], desc_str)