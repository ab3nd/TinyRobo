#!/usr/bin/python

#Go through all the data files and suggest validation/normalization stuff

#Get all csv files in the directory
files = [f for f in os.listdir('.') if os.path.isfile(f)]
	for f in files:
		import pdb; pdb.set_trace()

#For each file, read the file

#Check that each line has the same number of commmas

#Check that timestamps are monotonically increasing

#list all unique codes

