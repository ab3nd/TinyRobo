#!/usr/bin/python

#Prepare for anova for all participants
#Collects mean counts of each gesture for all participants in that condtion


import all_data_handler


def get_gesture_counts(participant):
	gestures = {"drag":0,
				"draw":0,
				"ui":0,
				"tap":0,
				"doubletap":0,
				"tripletap":0,
				"hold":0,
				"pinch":0,
				"rev_pinch":0,
				"lasso":0,
				"box":0,
				"voice":0,
				"other":0}

	#Get all the events
	events = []
	for task in participant["tasks"].keys():
		events.extend(participant["tasks"][task])

	for event in events:
	 	if event["event_type"] == "tap":
	 		#Taps need special handling, as they might be double, triple, or hold
	 		if event["hold"]:
	 			gestures["hold"] += 1
	 		elif event["count"] == 2:
	 			gestures["doubletap"] += 1
	 		elif event["count"] == 3:
	 			gestures["tripletap"] += 1
	 		else:
	 			gestures["tap"] += 1
		elif event["event_type"] == "drag":
			#Drags might be drag or might be draw
			if event["draw"] is None:
				gestures["drag"] += 1
			else:
				gestures["draw"] += 1
		elif event["event_type"] == "pinch":
			#pinch can be pinch or reverse
			if event["reverse"]:
				gestures["rev_pinch"] += 1
			else:
				gestures["pinch"] += 1
		elif event["event_type"] == "voice_command":
			gestures["voice"] += 1
		elif event["event_type"] == "ui":
			gestures["ui"] += 1
		elif event["event_type"] == "memo":
			#Don't do anything with memos
			pass
		elif event["event_type"] == "lasso":
			gestures["lasso"] += 1
		elif event["event_type"] == "box_select":
			gestures["box"] += 1
		elif event["event_type"] == "other":
			gestures["other"] += 1		
		else:
			#This is an error, some event type wasn't handled
			print event["event_type"]

	return gestures

def average_counts(counts):
	participant_count = len(counts.keys())
	totals = {}
	#Collect the total counts
	for p in counts.keys():
		p_count = counts[p]
		for gesture in p_count.keys():
			if gesture in totals.keys():
				totals[gesture] += p_count[gesture]
			else:
				totals[gesture] = p_count[gesture]
	#Average across participants
	for gesture in totals.keys():
		totals[gesture] = totals[gesture]/float(participant_count)

	return totals

if __name__ == "__main__":
	adh = all_data_handler.UserData()

	#Get the counts of a participant in each condition
	for condition in adh.conditionMap.keys():
		counts = adh.applyCondition(get_gesture_counts, condition)
		avg = average_counts(counts)
		print condition
		print avg
		print "---"	
