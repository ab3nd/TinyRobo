#!/usr/bin/python

#Prettify all the json files in the current dir

import json
import os

jsonfiles = [x for x in os.listdir("./") if x.endswith(".json")]
for file in jsonfiles:
	data = None
	with open(file, 'r') as infile:
		try:
			data = json.loads(infile.read())
		except ValueError:
			print "Value error loading {0}".format(file)
			raise
			
	with open(file, 'w') as outfile:
		outfile.write(json.dumps(data, sort_keys=True, indent=3))

