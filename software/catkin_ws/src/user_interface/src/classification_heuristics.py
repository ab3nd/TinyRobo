#!/usr/bin/python

#Collects a bunch of recognition routines for strokes to determine what kind of stroke it is, 
#e.g. line, arc, etc. Used by the various gesture detectors to classify strokes. 

import math

def distanceEvents(eventA, eventB):
	x1 = eventA.point.x
	y1 = eventA.point.y
	x2 = eventB.point.x
	y2 = eventB.point.y

	return distance(x1, y1, x2, y2)

def distance(x1, y1, x2, y2):
	return math.sqrt(pow(x1 - x2, 2) + pow(y1 -y2, 2))

def clamp(n, minn, maxn):
	return max(min(maxn, n), minn)

def get_centroid_angle(stroke):
	#Can't have an angle between less than two events
	if len(stroke.events) > 2:
		#Get the angle between the end points around the centroid
		#Start by getting the distances between the start and end events and the centroid
		x1 = stroke.events[0].point.x
		y1 = stroke.events[0].point.y
		x2 = stroke.centroid.x
		y2 = stroke.centroid.y
		d1 = distance(x1, y1, x2, y2)
		x1 = stroke.events[-1].point.x
		y1 = stroke.events[-1].point.y
		d2 = distance(x1, y1, x2, y2)
		#Distance between the start and end events
		d3 = distanceEvents(stroke.events[0], stroke.events[-1])
		#From the law of cosines and https://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
		#This should be in radians, math.acos is in radians, according to the docs
		#value is capped at 1 because numerical imprecision was causing math domain errors by feeding
		#acos values like -1.00000000002, which is, technically, out of range
		if (d1 * d2) == 0:
			#At least ine of the points was no distance from the centroid, so there
			#can't be an angle from the centroid to it
			return None
		capped = clamp((pow(d1, 2) + pow(d2, 2) - pow(d3, 2))/(2 * d1 * d2), -1, 1)
		angle = math.acos(capped)
		return angle
	return None

def getAvgCentroidDist(gesture):
    cX = gesture.centroid.x
    cY = gesture.centroid.y
    avg = 0.0
    for touch in gesture.events:
        avg += distance(cX, cY, touch.point.x, touch.point.y)
    avg = avg / len(gesture.events)
    return avg


class Enum(set):
	def __getattr__(self, name):
		if name in self:
			return name
		raise AttributeError

GestureType = Enum(["LINE", "CIRCLE", "ARC"])#, "TAP", "DOUBLE_TAP", "TRIPLE_TAP", "CMD_GO"])


def get_class(stroke):
	angle = get_centroid_angle(stroke)
	if angle < 1:
		return GestureType.CIRCLE
	elif angle < 2.5:
		return GestureType.ARC
	elif angle is not None:
		return GestureType.LINE
	else:
		return None

def is_line(stroke):
	return get_class(stroke) == GestureType.LINE

def is_arc(stroke):
	return get_class(stroke) == GestureType.ARC

def is_circle(stroke):
	return get_class(stroke) == GestureType.CIRCLE

def is_tap(stroke):
	if len(stroke.events) < 10:
		if getAvgCentroidDist(stroke) < 10:
			#Less than 10 events, close in position in space
			return True
	return False
