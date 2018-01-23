#!/usr/bin/python
# Quantify the badness of using a "null" code to represent missing data when calculating Cohen's k

from sklearn.metrics import cohen_kappa_score
import random
import copy

#Shouldn't really matter
items = 20

#Does number of labels affect this?
for labels in range(2,10):
	print "--- {0} labels ---".format(labels)
	#Generate two lists of artifical codes that are the same
	x1 = [random.randint(1,labels) for _ in range(items)]
	x2 = copy.deepcopy(x1)
	x3 = copy.deepcopy(x2)
	
	for ii in range(len(x3)):
		ck13 = cohen_kappa_score(x1,x3)
		ck23 = cohen_kappa_score(x2,x3)
		diff = abs(ck13-ck23)
		print "{0}\t{1}\t{2}".format(ck13, ck23, diff)		

		#Now try making one of them wronger, while making the other have more nulls
		if x1[ii] > 1:
			x1[ii] -= 1
		else:
			x1[ii] += 1

		#-1 is our "null" code
		x2[ii] = -1
	
