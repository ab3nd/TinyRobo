#!/usr/bin/python

#Given a LaTeX file, find all the section, subsection, subsubsection etc. lines and put a label on 
#each one that is labeled as the section heading (with spaces and punctuation removed)

import sys
import re
import tempfile
import shutil

#Get the input file
infile = sys.argv[1]

#Regex to match section headings and get the title
re_section = re.compile("section\{([^\}]*)\}")
#Regex to check if we already have a label on this line
re_label = re.compile("label\{")

with tempfile.NamedTemporaryFile(dir='.') as tmp, open(infile, 'r') as latex_file:
	for line in latex_file:
		sec_match = re.search(re_section, line)
		#We found a section and it doesn't have a label
		if sec_match and not re.search(re_label, line):
			head = sec_match.group(1)
			#Replace non alphanumeric characters with underscore
			head = re.sub("\W", "_", head)
			#Put a "section:" heading so they don't get confused with e.g. figures
			head = " \label{section:"+head+"}\n"
			new_line = line.strip() + head
			#Write our nice fancy new line
			tmp.write(new_line)
		else:
			#Write the old line
			tmp.write(line)
	#uses copy rather than move, so that the tempfile is still there to get
	#deleted when the with clause ends and the tempfile goes out of scope
	shutil.copy(tmp.name, infile)

