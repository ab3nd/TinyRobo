#!/usr/bin/python

#Combine all the json files into one huge json file
# map of numbers to participants, then each participant is the data from that file

import json

#Generated with for file in *.json; do echo \"$file\"\,; done
files = ["p_10_ams.json",
		"p_11_ams.json",
		"p_12_ams.json",
		"p_13_ams.json",
		"p_14_ams.json",
		"p_15_ams.json",
		"p_16_ams.json",
		"p_17_ams.json",
		"p_18_ams.json",
		"p_19_ams.json",
		"p_1_ams.json",
		"p_20_ams.json",
		"p_21_ams.json",
		"p_22_ams.json",
		"p_23_ams.json",
		"p_24_ams.json",
		"p_25_ams.json",
		"p_26_ams.json",
		"p_27_ams.json",
		"p_28_ams.json",
		"p_29_ams.json",
		"p_2_ams.json",
		"p_30_ams.json",
		"p_3_ams.json",
		"p_4_ams.json",
		"p_5_ams.json",
		"p_6_ams.json",
		"p_7_ams.json",
		"p_8_ams.json",
		"p_9_ams.json"
		]

outfileName = "all_participants.json"

data = {}
#Load each file
for infileName in files:
	#Get participant from file name
	participant = infileName.split("_")[1]
	with open(infileName, 'r') as infile:
		data[participant] = json.loads(infile.read())

#Prettyprint and dump it
with open(outfileName, 'w') as outfile:
	outfile.write(json.dumps(data, sort_keys=True, indent=3))