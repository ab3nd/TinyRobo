#!/usr/bin/python

#Finds duplicate lines in a LaTeX file and comments out the second one

import sys

#No need for fancy command line processing
infile = sys.argv[1]

with open(infile, 'r') as inputFile:
	for line in inputFile:
		print line