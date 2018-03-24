#!/usr/bin/python

#Finds duplicate lines in a LaTeX file and comments out the second one

import sys
import re

#No need for fancy command line processing
infile = sys.argv[1]

seenLines = {}


with open(infile, 'r') as inputFile:
	for line in inputFile:
		if line in seenLines.keys() and re.search("\S", line) :
			#We've seen this line before
			print "%", line,
		elif re.search("cite{", line):
			#Bad/old cites
			print "%", line,
		else:
			seenLines[line] = 1
			print line,