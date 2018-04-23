#!/usr/bin/python

# Parse the user survey into a json file for later use

import json
import csv

# The CSV file has effectively three header rows. The first is the question ID, 
# the second is the text of the question, and the third is doublequoted (as in
# two sets of double quotes) versions of the question IDs. I'm not sure what 
# the last one is supposed to be useful for. 

#Import the csv file
fname = "Swarm+Test+Drive_December+4%2C+2017_16.53.csv"
with open(fname, 'rb') as csvfile:
	reader = csv.reader(csvfile)
	#Get the question IDs and the question texts
	q_IDs = reader.next()
	q_text = reader.next()
	import_IDs = reader.next()

	#Lookup table for getting question text from question IDs
	text_lookup = {k:v for k, v in zip(q_IDs, q_text)}
	all_particpants = {}

	try:
		participant = reader.next()
	except StopIteration:
		participant = None
		
	while participant is not None:

		#data in a dict keyed by question ids
		participant_data = {k:v for k, v in zip(q_IDs, participant)}

		#Store keyed by ID
		all_particpants[participant_data["Q1"]] = participant_data

		#Get the next participant, or quit this loop if we're done
		try:
			participant = reader.next()
		except StopIteration:
			participant = None

	#Now we have all the data loaded, dump it to a nicely formatted JSON file
	#including the lookup table
	data = {}
	data["lookup"] = text_lookup
	data["participants"] = all_particpants
	with open("./survey.json", 'w') as outfile:
		outfile.write(json.dumps(data, sort_keys=True, indent=3))

	