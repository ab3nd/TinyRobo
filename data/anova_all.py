#!/usr/bin/python

#Prepare for anova for all participants
#Collects mean counts of each gesture for all participants in that condtion


import all_data_handler


def get_gesture_counts(participant):
	import pdb; pdb.set_trace()

if __name__ == "__main__":
	adh = all_data_handler.UserData()
	
	#Get the counts of a participant in the single-robot condition
	counts = adh.applyCondition(get_gesture_counts, "one")