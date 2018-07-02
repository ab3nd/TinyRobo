#!/usr/bin/python

#IPython seems to be unreliable, I'm getting different outputs from running the same 
#cell over and over, so there is some sort of invisible state getting saved that is 
#screwing up calculations. 

#Preamble stuff, loading up libraries and convenience functions
import all_data_handler
import pandas
import statsmodels
import statsmodels.api as sm
from statsmodels.formula.api import ols
import itertools
import seaborn as sb
import scipy.stats as ss #For ss.f_oneway() ANOVAs
import re
import matplotlib.pyplot as plt
import numpy as np
import copy

adh = all_data_handler.UserData()
per_task_df = pandas.DataFrame(adh.toPandas())
col_list = list(per_task_df)
col_list.remove("user")
col_list.remove("condition")
col_list.remove("task")

#Given the object list of a gesture from the coding, return a list of letters representing normalized 
#values for the objects of the gesture
def tag_object(original):
	#This use of strip is to prevent quotes from messing up regex matches
	original = " ".join(original).strip("\"")
	tags = []
	#Matches robots, robot, bot, bots, etc. 
	robots = re.compile("bot|group|swarm|orange|red", re.I)
	crate = re.compile("crate|wood", re.I)
	targetA = re.compile("area a|box a| a$|to a,|^a$", re.I)
	targetB = re.compile("area b|box b| b$|to b,|^b$", re.I)
	whitespace = re.compile("whitespace|ground|screen|white area", re.I)
	
	toCheck = [(robots, "r"), (crate, "c"), (targetA, "a"), (targetB, "b"), (whitespace, "w")]
	for compiled, tag in toCheck:
		if re.search(compiled, original):
			tags.append(tag)
	return tags

def count_select_taps(participant):
	#Dict of task id to count of taps on robots
	counts = {}
	for task_id in participant["tasks"].keys():
		#Default to no counts of select taps
		counts[task_id] = 0
		events = participant["tasks"][task_id]
		#Flip through all the events
		for event in events:
			if event["event_type"] == "memo":
				#This event is a memo, not a user gesture
				continue
			elif "example" in event.keys() and event["example"] == True:
				#This event is an example, don't count it
				continue
			else:
				#We have an event, check if it's a tap on a robot
				if event["event_type"] == "tap":
					#Taps always have an object
					tags = tag_object(event["objects"])
					if 'r' in tags:
						counts[task_id] += 1
	return counts

def count_group_select(participant):
	#Dict of task id to count of taps on robots
	counts = {}
	for task_id in participant["tasks"].keys():
		#Default to no counts of select taps
		counts[task_id] = 0
		events = participant["tasks"][task_id]
		#Flip through all the events
		for event in events:
			if event["event_type"] == "memo":
				#This event is a memo, not a user gesture
				continue
			elif "example" in event.keys() and event["example"] == True:
				#This event is an example, don't count it
				continue
			else:
				if (event["event_type"] == "lasso" or event["event_type"] == "box_select"):
					tags = tag_object(event["objects"])
					#We have both an event and a set of tags
					if 'r' in tags:
						counts[task_id] += 1
	return counts

def count_box_select(participant):
	#Dict of task id to count of taps on robots
	counts = {}
	for task_id in participant["tasks"].keys():
		#Default to no counts of select taps
		counts[task_id] = 0
		events = participant["tasks"][task_id]
		#Flip through all the events
		for event in events:
			if event["event_type"] == "memo":
				#This event is a memo, not a user gesture
				continue
			elif "example" in event.keys() and event["example"] == True:
				#This event is an example, don't count it
				continue
			else:
				if event["event_type"] == "box_select":
					tags = tag_object(event["objects"])
					if 'r' in tags:
						counts[task_id] += 1
	return counts

def count_lasso_select(participant):
	#Dict of task id to count of taps on robots
	counts = {}
	for task_id in participant["tasks"].keys():
		#Default to no counts of select taps
		counts[task_id] = 0
		events = participant["tasks"][task_id]
		#Flip through all the events
		for event in events:
			if event["event_type"] == "memo":
				#This event is a memo, not a user gesture
				continue
			elif "example" in event.keys() and event["example"] == True:
				#This event is an example, don't count it
				continue
			else:
				if event["event_type"] == "lasso":
					#Tag the objects
					tags = tag_object(event["objects"])
					if 'r' in tags:
						#Event was a lasso, and it was on a robot
						counts[task_id] += 1
	return counts

#Returns a dict of task IDs to count of total events
def count_events(participant):
	counts = {}
	for task_id in participant["tasks"].keys():
		#Default to no counts of select taps
		counts[task_id] = 0
		#Flip through all the events
		events = participant["tasks"][task_id]
		for event in events:
			if event["event_type"] == "memo":
				#This event is a memo, not a user gesture
				continue
			elif "example" in event.keys() and event["example"] == True:
				#This event is an example, don't count it
				continue
			else:
				#Not one of the skip cases, so count it
				counts[task_id] += 1
	return counts

#Prints out the per-condition counts, one entry for each user in that condition
#Also gets the mean and standard deviation for the condition
def user_counts(user_counts):
	per_cond = {}
	#For each user, maintain a running total
	for pid in user_counts.keys():
		#Get their condition
		cond = adh.IdToCondition(pid)[0]
		counts = user_counts[pid].values()
		per_user_total = sum(counts)
		if cond in per_cond.keys():
			per_cond[cond].append(per_user_total)
		else:
			per_cond[cond] = [per_user_total]
	
	for cond in per_cond.keys():
		
		total = sum(per_cond[cond])
		mean = np.mean(per_cond[cond])
		std_dev = np.std(per_cond[cond])
		print cond, per_cond[cond]
		print "total:{} mean:{} std dev:{}".format(total, mean, std_dev)
		print

#Given a dict of user IDs to dicts of task ids to event counts
#return a dict of user IDs to lists of event counts for the common tasks
def get_matched_tasks(counts):
	#Get a list of the task names that every condition has in common
	common_names = []
	for name in adh.taskMap.keys():
		if all(adh.taskMap[name].values()):
			common_names.append(name)
	common_names.sort()

	#dict of user ids to a list of counts, index of counts is task number
	matched_tasks = {}
	for user in counts.keys():
		common_counts = []
		for task in common_names:
			#print user, adh.IdToCondition(user), task, adh.taskNameToNumber(task, user)
			#Get the count at the task number for this common name
			common_counts.append(counts[user][str(adh.taskNameToNumber(task, user))])
		matched_tasks[user] = common_counts
	return matched_tasks

#Given a dict of conditions to lists of samples
#Do an all-pairs 1-way ANOVA on the samples
def all_pairs_f(d):
	for k1, k2 in itertools.combinations(d.keys(), 2):
		print k1, k2
		print ss.f_oneway(d[k1], d[k2])
		print

#Given a dict of users to lists of samples
#Return a dict of conditions to a list of all the samples for users in that condition 
def make_condition_dict(user_samples):
	cond_dict = {}
	for user in user_samples.keys():
		#Get the condition for this user
		cond = adh.IdToCondition(user)[0]
		#If we already have a set of samples, extend it, otherwise, create it
		if cond in cond_dict.keys():
			cond_dict[cond].extend(user_samples[user])
		else:
			cond_dict[cond] = copy.deepcopy(user_samples[user])
	return cond_dict


###################### Per user Normalization ############################################

#Set up dicts of user ids to dicts of task ids to various types of counts
all_select_taps = adh.apply(count_select_taps)
all_group_selects = adh.apply(count_group_select)
all_lassos = adh.apply(count_lasso_select)
all_box = adh.apply(count_box_select)
all_gesture_counts = adh.apply(count_events)

#Filter user gesture counts, user tap-as-select counts, and user group select counts to common tasks only
common_select_tap = get_matched_tasks(all_select_taps)
common_group_select = get_matched_tasks(all_group_selects)
common_gesture_counts = get_matched_tasks(all_gesture_counts)

#For each user, get the sum of their gesture use across all the common tasks. This is the user's "verbosity".
#The verbosity is used to normalize the raw counts of gestures so that users that make a lot of gestures don't
#dominate the analysis. 
total_gesture_counts = {k:sum(v) for (k,v) in common_gesture_counts.items()}

#Normalize tap-as-select for matched tasks
#This is a list comprehension that does the norming (divide each task by the appropriate entry in the total counts)
#inside of a dictionary comprehension that operates over all users
norm_select_tap = {k:[float(count)/float(total_gesture_counts[k]) for count in v] for (k,v) in common_select_tap.items()}
#Normalize group select for matched tasks
norm_group_select = {k:[float(count)/float(total_gesture_counts[k]) for count in v] for (k,v) in common_group_select.items()}

#Set up dictionaries by condition for all-pairs ANOVA for tap-as-select and group select
select_tap_by_cond = make_condition_dict(norm_select_tap)
group_select_by_cond = make_condition_dict(norm_group_select)

#And do our ANOVAS
print "Tap as Select, per-user normalization"
all_pairs_f(select_tap_by_cond)

print "Group Select, per-user normalization"
all_pairs_f(group_select_by_cond)



# ####################### All tasks, 10, 100, 1000 cases ##############################

# #Convert the dictionary of user id to dictionary of task to count to a simple list of counts
# #The list is ordered by task
def tasks_to_list(tasks):
	counts = []
	for key in sorted(tasks.iterkeys()):
		counts.append(tasks[key])
	return counts

# #For each user, get the sum of their gesture use across all the tasks. As above, this gets used as verbosity.
total_gesture_counts = {k:sum([count for count in v.values()]) for (k,v) in all_gesture_counts.items()}

# #Convert the dicts of per-task dicts of counts into a dict of ordered list of counts
# #The keys at the top level remain the user ids
tap_select_counts = {k:tasks_to_list(v) for (k,v) in all_select_taps.items()}
group_select_counts = {k:tasks_to_list(v) for (k, v) in all_group_selects.items()}

# #Normalize the counts by the user's total gestures
norm_select_tap = {k:[float(count)/float(total_gesture_counts[k]) for count in v] for (k,v) in tap_select_counts.items()}
norm_group_select = {k:[float(count)/float(total_gesture_counts[k]) for count in v] for (k,v) in group_select_counts.items()}

#Set up dictionaries by condition for all-pairs ANOVA for tap-as-select and group select
select_tap_by_cond = make_condition_dict(norm_select_tap)
group_select_by_cond = make_condition_dict(norm_group_select)

#Get rid of the conditions we're not checking (the don't have the same number of samples)
del select_tap_by_cond["one"]
del select_tap_by_cond["unknown"]
all_pairs_f(select_tap_by_cond)

#Get rid of the conditions we're not checking (the don't have the same number of samples)
del group_select_by_cond["one"]
del group_select_by_cond["unknown"]
all_pairs_f(group_select_by_cond)

################################## Total Counts ########################################



def total_by_condition(by_user):
	counts_by_cond = make_condition_dict(by_user)
	for cond in counts_by_cond.keys():
		print cond
		print "\tTotal: {}".format(sum(counts_by_cond[cond]))

# #Filter user gesture counts, user tap-as-select counts, and user group select counts to common tasks only
common_select_tap = get_matched_tasks(all_select_taps)
common_group_select = get_matched_tasks(all_group_selects)

#Display the total count of group selection gestures per condition
print "\n-- group select, all tasks --"
total_by_condition(group_select_counts)

#Display the total amount of group selection gestures within the common tasks, per condition
print "\n-- group select, common tasks --"
total_by_condition(common_group_select)

#Display the total amount of tap selection gestures per condition
print "\n-- tap select, all tasks --"
total_by_condition(tap_select_counts)

#Display the total amount of tap selection gestures within the common tasks, per condition
print "\n-- tap select, common tasks --"
total_by_condition(common_select_tap)

############################## Per-Task Normalization ######################################

#Filter user gesture counts, user tap-as-select counts, and user group select counts to common tasks only
common_select_tap = get_matched_tasks(all_select_taps)
common_group_select = get_matched_tasks(all_group_selects)
common_gesture_counts = get_matched_tasks(all_gesture_counts)

#The get_matched_tasks function does a sort on task names, so the lists produced are ordered by task. 
#This means I can get away with zipping the common gesture counts and e.g. the common select taps, and then
#doing the division to normalize in a list comprehension
#Or I could, if it were not for the fact that sometimes the gesture count is 0 (rarely, some users made no gestures)
norm_select_tap = {k:[float(a)/float(b) if a != 0 else 0.0 for a,b in zip(common_select_tap[k], common_gesture_counts[k])] for k in common_select_tap.keys()}
#Normalize group select for matched tasks
norm_group_select = {k:[float(a)/float(b) if a != 0 else 0.0 for a,b in zip(common_group_select[k], common_gesture_counts[k])] for k in common_group_select.keys()}

#Set up dictionaries by condition for all-pairs ANOVA for tap-as-select and group select
select_tap_by_cond = make_condition_dict(norm_select_tap)
group_select_by_cond = make_condition_dict(norm_group_select)

#And do our ANOVAS
print "Tap as Select, per-task normalization"
all_pairs_f(select_tap_by_cond)

print "Group Select, per-task normalization"
all_pairs_f(group_select_by_cond)


############################## Tap Sequence Length ##################################
#Looking at lengths of sequences of taps
def tap_sequences(participant):
	#Dict of task id to count of taps on robots
	counts = {}
	for task_id in participant["tasks"].keys():
		#Set up an empty list for this task
		counts[task_id] = []
		events = participant["tasks"][task_id]
		#Flip through all the events
		temp = []
		for event in events:   
			if event["event_type"] == "memo":
				#This event is a memo, not a user gesture
				continue
			elif "example" in event.keys() and event["example"] == True:
				#This event is an example, don't count it
				continue
			else:
				#We have an event, check if it's a tap on a robot
				if event["event_type"] == "tap":
					#Taps always have an object
					tags = tag_object(event["objects"])
					if 'r' in tags:
						#This is a tap on a robot, add it to the temp list
						temp.append(event)
					else:
						#is a tap, not on a robot, so end the sequence
						#This is the end of the sequence, push the temp list into the event list
						if len(temp) > 0:
							counts[task_id].append(temp)
						#Reset the temp list for the next sequence
						temp = []
				else:
					#This is the end of the sequence, push the temp list into the event list
					if len(temp) > 0:
						counts[task_id].append(temp)
					#Reset the temp list for the next sequence
					temp = []
	return counts

#Get the tap sequences 
seqs = adh.apply(tap_sequences)

#Convert to per-user, per-task lengths
seq_lens = {}
for uid in seqs.keys():
	seq_lens[uid] = {}
	for tid in seqs[uid].keys():
		seq_lens[uid][tid] = [len(taps) for taps in seqs[uid][tid]]

#Just get some per-user totals
raw_counts = {}
total = 0
for uid in seq_lens.keys():
	raw_counts[uid] = 0
	#Sum up over all the tasks
	for tid in seq_lens[uid].keys():
		raw_counts[uid] += sum(seq_lens[uid][tid])
	total += raw_counts[uid]

print total
print raw_counts

#Put it in a dictionary by condition
#Make condition dict expects everything in arrays already
raw_counts = {k:[v] for k,v in raw_counts.items()}
print make_condition_dict(raw_counts)

#Convert each user's per task counts into a list of counts
by_user = {}
for user in seq_lens.keys():
	sequences = []
	for task in seq_lens[user].keys():
		sequences.extend(seq_lens[user][task])
	by_user[user] = sequences
cond_seqs = make_condition_dict(by_user)
print cond_seqs

total = 0
for c in cond_seqs.keys():
	c_str = (" " * (10-len(c))) + c
	seq_count = len(cond_seqs[c])
	total += sum(cond_seqs[c])
	mean_len = float(sum(cond_seqs[c]))/float(seq_count)
	std_dev = np.std(cond_seqs[c])
	print "{} \t Count: {}\t Mean: {:.4f}\t Std. Dev.: {:.4f}".format(c_str, seq_count, mean_len, std_dev)
print "Total tap selects: {}".format(total)