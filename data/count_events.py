#!/usr/bin/python

# count all the events in a json file 
# To call on every file in a directory, you can do this:
# for file in `ls *.json`; do outfile=$(basename "$file"); outfile="${outfile%.*}"; echo $file > $outfile.txt; ./count_events.py $file >> $outfile.txt; done

import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("inFile", nargs = 1, help="File to print")
args = parser.parse_args()

#Get the file name
fname = args.inFile[0]

with open(fname, "r") as infile:
	data = json.loads(infile.read())
	for task in sorted(data['tasks'].keys(), key=int):
		events = data['tasks'][task]
		#Only care about events with times
		events = [x for x in events if 'time' in x.keys()]
		for event in sorted(events, key=lambda x: float(x['time'])):
			print event