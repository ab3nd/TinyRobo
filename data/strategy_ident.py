#!/usr/bin/python

#Normalize use of words like "crate", "whitespace", "robots", etc. in object declarations
#Possibly through tagging gestures with a normalized tag rather than changing description

#Given a set of user gestures with normalized tags, build a stack representing the data

#Possibly reduce the stack by compacting neighboring matching gestures with the same object

#Calculate edit distance on stacks to de-duplicate strategies (based on observation)
#This is a point where I can encode my heuristics

#Cluster stacks based on edit distance. How does it correlate with my tagged strategies
#This is a point where I can try to resist my interest in machine learning

import math
import nltk
import all_data_handler

users = all_data_handler.UserData()


class Ngrammer(object):
	def __init__(self):
		self.texts = []
		#memoization of n-grams, keyed by N
		self.grams = {}

	#For all the texts, build the n-grams
	def get_n_grams(self, n):
		grams = {}
		if n in self.grams.keys():
			return self.grams[n]
		else:
			#This is where we do the actual N-gramming
			for text in self.texts:
				text = text.split(" ")
				for gram in zip(*[text[i:] for i in range(n)]):
					if gram in grams.keys():
						grams[gram] += 1
					else:
						grams[gram] = 1
		print "{0} {1}-grams".format(len(grams.keys()), n)
		self.grams[n] = grams
		return grams

	#Add a single line of text to this grammer
	def add_text(self, t):
		if t is not None:
			#Regularize to lowercase and strip some punctuation
			t = " ".join(t)
			t = t.lower()
			t = t.replace("\"", "")
			t = t.replace("/", " ")
			t = t.replace("\'", "")
			t = t.replace("(", "")
			t = t.replace(")", "")
			t = t.replace(",", "")
			#TODO They arrive as lists... I guess joining and splitting later is inefficent
			self.texts.append(t)
			#Adding text invalidates any stored n-grams
			self.grams = {}


def save_ngrams():
	#Build seperate sets of N-grams for objects, description, and draw commands
	objGrams = Ngrammer()
	drawGrams = Ngrammer()
	descGrams = Ngrammer()

	for participant in users.data.keys():
		for task in users.data[participant]["tasks"].keys():
			for event in users.data[participant]["tasks"][task]:
				#Skip memos
				if event["event_type"] == "memo":
					continue

				#Get objects for most events
				if "objects" in event.keys():
					obj = event["objects"]
					objGrams.add_text(obj)

				#Get description for "other" events, memo was already skipped
				if "description" in event.keys():
					desc = event["description"]
					descGrams.add_text(desc)

				#Get what was drawn for drag-draw events
				if "draw" in event.keys():
					draw = event["draw"] #TODO can be None, handle gracefully
					drawGrams.add_text(draw)

	#Loaded everything, generate word list (1-grams), 2-grams, and 3-grams
	for grammer, name in [(objGrams, "objects"), (drawGrams, "draw"), (descGrams, "desc")]:
		oneGrams = grammer.get_n_grams(1)
		twoGrams = grammer.get_n_grams(2)
		threeGrams = grammer.get_n_grams(3)

		#Print them out nicely
		fName = "{}_n-grams.txt".format(name)
		with open (fName, 'w') as outfile:
			#Write a file header
			outfile.write("{}\n".format(name))
			outfile.write("=" * len(name))
			outfile.write("\n\n")
			for g, n in [(oneGrams,1), (twoGrams,2), (threeGrams,3)]:
				#Write a count header
				outfile.write(str(n))
				outfile.write("\n-\n\n")
				#Write the n-grams, sorted by count
				for key, value in sorted(g.iteritems(), key=lambda (k,v): (v,k)):
					#Skip hapax legomenae?
					if value == 1:
						continue
					outfile.write("{}\t\t".format(value))
					outfile.write(" ".join(key))
					outfile.write("\n")
				outfile.write("\n\n")

class TFIDF(object):
	def __init__(self):
		#Count of docs containing a word
		self.doc_counts = {}
		self.docs = 0.0

	def add_text(self, t):
		#We're adding a new doc
		self.docs += 1.0
		#Get all the unique words in this text
		uniques = list(set(nltk.word_tokenize(t)))
		for u in uniques:
			if u in self.doc_counts.keys():
				self.doc_counts[u] += 1
			else:
				self.doc_counts[u] = 1

	def get_tfidif(self, t):
		word_counts = {}
		#Count occurances of each word in this text
		words = nltk.word_tokenize(t)
		for w in words:
			if w in word_counts.keys():
				word_counts[w] += 1
			else:
				word_counts[w] = 1
		#Calculate the TF-IDF for each word
		tfidfs = []
		for w in words:
			#Word count is either 0 (It's in no docs), or the count
			w_docs = 0
			if w in self.doc_counts.keys():
				w_docs = self.doc_counts[w]

			#the 1 is to avoid div/zero for previously unseen words
			idf = math.log(self.docs/(1+w_docs))
			tf = word_counts[w]
			tfidfs.append((w, tf * idf))
		return tfidfs

def print_tfidf():
	#collect all the descriptions, to TF/IDF on them
	tfidfer = TFIDF()
	descriptions = []
	for participant in users.data.keys():
			for task in users.data[participant]["tasks"].keys():
				for event in users.data[participant]["tasks"][task]:
					#Skip memos
					if event["event_type"] == "memo":
						continue

					#Get description for "other" events, memo was already skipped
					if "description" in event.keys():
						desc = " ".join(event["description"])
						#Strip quotes and lowercase
						desc = desc.replace("\"", "")
						desc = desc.lower()

						#Save the modified description and add to the TF-IDF 
						descriptions.append(desc)
						tfidfer.add_text(desc)

	#Now print them all out
	for d in descriptions:
		tagged = tfidfer.get_tfidif(d)
		for word in tagged:
			print " {0}({1:.3f})".format(word[0], word[1]),
		print "\n"

import re

def tag_object(original):
	tags = []
	#Matches robots, robot, bot, bots, etc. 
	robots = re.compile("bot|group|swarm|orange|red", re.I)
	crate = re.compile("crate", re.I)
	targetA = re.compile("area a|box a| a$", re.I)
	targetB = re.compile("area b|box b| b$", re.I)
	whitespace = re.compile("whitespace|ground|screen", re.I)
	
	toCheck = [(robots, "ROBOTS"), (crate, "CRATE"), (targetA, "A"), (targetB, "B"), (whitespace, "WHITESPACE")]
	for compiled, tag in toCheck:
		if re.search(compiled, original):
			tags.append(tag)
	return tags

counts = {}
for participant in users.data.keys():
		for task in users.data[participant]["tasks"].keys():
			for event in users.data[participant]["tasks"][task]:
				#Skip memos
				if event["event_type"] == "memo":
					continue

				#Get objects for most events
				if "objects" in event.keys():
					obj = " ".join(event["objects"])
					obj = obj.replace("\"", "")
					obj = obj.lower()

					if obj in counts.keys():
						counts[obj] += 1
					else:
						counts[obj] = 1

for key, value in sorted(counts.iteritems(), key=lambda (k,v): (v,k)):
	tags = tag_object(key)
	print "{}\t{}\t{}".format(value, tags, key)