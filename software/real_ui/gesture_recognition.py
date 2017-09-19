#Gesture recognition classes and methods

import math

class Enum(set):
    def __getattr__(self, name):
        if name in self:
            return name
        raise AttributeError

GestureType = Enum(["LINE", "POLY", "ARC", "TAP", "DOUBLE_TAP", "TRIPLE_TAP", "CMD_GO"])

class GestureRecognizer():
    def __init__(self):
        #Gestures, stored by UID
        self.pastGestures = {}

    def getCentroid(self, gesture):
        centroidX = 0
        centroidY = 0
        for touch in gesture:
            centroidX += touch.pos[0]
            centroidY += touch.pos[1]
        centroidX = centroidX/len(gesture)
        centroidY = centroidY/len(gesture)
        return centroidX, centroidY

    def getAvgCentroidDist(self, gesture):
        cX, cY = self.getCentroid(gesture)
        avg = 0
        for touch in gesture:
            avg += self.distance(cX, cY, touch.pos[0], touch.pos[1])
        avg = avg / len(gesture)
        return avg

    def distance(self, aX, aY, bX, bY):
        if (aX == bX) and (aY == bY):
            return 0
        try:
            return math.sqrt((pow(aX,2) - pow(bX, 2)) + (pow(aY, 2) - pow(bY, 2)))
        except ValueError:
            print aX, aY, bX, bY
            return 0

    #Accepts a gesture, tries to recognize it, returns None if the gesture isn't recognized
    def recognize(self, gesture):
        #This isn't something you should just be calling, it's the superclass for the others
        return None

class TapRecognizer(GestureRecognizer):
    def recognize(self, gesture):
        if len(gesture) < 10:
            if self.getAvgCentroidDist(gesture) < 10:
                #Less than 10 events, close in position in space
                return True
        return False

class DoubleTapRecognizer(GestureRecognizer):
    def recognize(self, gesture):
    	return any([x.is_double_tap for x in gesture])

class TripleTapRecognizer(GestureRecognizer):
    def recognize(self, gesture):
        return any([x.is_triple_tap for x in gesture])    	    	

class RecognitionEngine():
	def __init__(self):
		self.recognizers = []

	def addRecognizer(self, r):
		self.recognizers.append(r)

	def recognize(self, gesture):
		#At present, this is just whichever recognizer gets it first. 
		#Arbitrating ambiguity can be handled later
		for recognizer in self.recognizers:
			output = recognizer.recognize(gesture)
			print output
		return output


def getRecognizer():
	recEng = RecognitionEngine()
	recEng.addRecognizer(TapRecognizer())
	recEng.addRecognizer(DoubleTapRecognizer())
	recEng.addRecognizer(TripleTapRecognizer())
	#recEng.addRecognizer(LineRecognizer())
	#recEng.addRecognizer(ArcRecognizer())
	#recEng.addRecognizer(PolyRecognizer())
	#recEng.addRecognizer(GoRecognizer())
	return recEng