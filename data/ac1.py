#!/usr/bin/python

#An implementation of Gwet's AC1 in python
# https://gist.github.com/mbq/5756760 has a version in R, for un-weighted AC1
# https://github.com/jmgirard/mReliability/wiki/Gwet%27s-gamma-coefficient has a description
# of the formulas for weighted AC1 with multiple raters and multiple categories

import numpy as np


def count_votes(a, categories):
	result = np.zeros((1,categories))
	for vote in a:
		#Skip missing votes
		if vote == 0:
			continue
		#Increment the count of this vote
		result[0][vote-1] += 1
	return result

def gwetAC1(ratings, categories = 5):

	#For nominal data, the weight matrix is the identity matrix, full credit for 
	#matches and no credit for non-matches
	nominal_weights = np.identity(categories)

	items = len(ratings[0])

	#Build the r_il table. r_il is the number of raters that voted object i to be of 
	#category l. 
	r_il = np.zeros((items, categories - 1))
	for index, col in enumerate(ratings.T):
		r_il[index] = count_votes(col, categories -1)


	#calculate pi_k, the mean number of rater votes for each class over the objects
	votes_per_event = np.sum(r_il, axis=1, keepdims=True)
	p_ik = np.mean(r_il/votes_per_event, axis=0)

	#calculate p_e, the expected chance agreement
	p_e = np.sum(p_ik * (1-p_ik))/(len(p_ik)-1)

	#remove objects that got votes from only one rater
	r_il_clear = None		
	for row in r_il:
		if np.sum(row) > 1:
			if r_il_clear is None:
				r_il_clear = np.array([row])
			else:
				r_il_clear = np.append(r_il_clear, [row], axis=0)

	#calculate p_a on the cleared set
	r_i = np.sum(r_il_clear, axis = 1)
	
	p_a = np.mean(np.sum((r_il_clear * (r_il_clear - 1)), axis = 1)/(r_i * (r_i -1)))

	#calculate gamma and return it
	return (p_a - p_e)/(1 - p_e) 


# I think the implementation above is correct, so now I need some test data

#Use zero for no rating, np.nan gave issues
data = np.array([[1,1,1,1],[1,1,1,1]])
print gwetAC1(data)

data = np.array([[1,1,1,1],[1,1,1,2]])
print gwetAC1(data)

data = np.array([[1,1,1,1],[1,1,2,2]])
print gwetAC1(data)

data = np.array([[1,1,1,2],[1,1,1,3]])
print gwetAC1(data)