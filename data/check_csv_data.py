#!/usr/bin/python

import os 
import csv
import re

#Go through all the data files and suggest validation/normalization stuff

#Store counts of all of the codes
all_codes={}

#for putting spaces before parens, matches a non-whitespace before an opening paren
space_paren = re.compile("([\S])\(")

#for putting spaces around arrows
space_arrow = re.compile("([\s]?)->([\s]?)")

#for removing anything in parens
strip_paren = re.compile("\([^\)]*\)")

def add_code(code):
	#break up at semicolons
	codes = code.split(';')
	for code in codes:
		code = code.strip()
		#put a single space before parens
		code = re.sub(space_paren, lambda x: "{0} (".format(x.group(1)), code)

		#put spaces around arrows
		code = re.sub(space_arrow, lambda x: "{0}->{1}".format(x.group(1) if x.group(1) else " ", x.group(2) if x.group(2) else " "), code)		

		#Drop targets (in parens) and only log the code
		code = re.sub(strip_paren, "", code)
		code = code.strip()

		#Update the counts of all codes
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
#for code in all_codes.keys():
#	print code, all_codes[code]
codeList = sorted(zip(all_codes.values(),all_codes.keys()), key=lambda item:item[0])
for item in codeList:
	print "{0}\t{1}".format(item[0], item[1])