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
        self.name = "GestureRecognizer"

    def getCentroid(self, gesture):
        #This has problems with non-polygons, e.g. a line with a small but very 
        #dense with points scribble at one end. The centroid would be expected 
        #to be the middle, but the mass of points at the end causes the centroid 
        #to be pulled out towards that end.
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

    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def getEndAngle(self, gesture):
        #Get coordinates for start and end of gesture, and gesture centroid
        x1 = gesture[0].x
        y1 = gesture[0].y
        x2 = gesture[-1].x
        y2 = gesture[-1].y
        xc, yc = self.getCentroid(gesture)
        d1 = distance(x1, y1, xc, yc) #Distance from start to centroid
        d2 = distance(x2, y2, xc, yc) #Distance from end to centroid
        d3 = distance(x1, y1, x2, y2) #Distance from start to end
        if d1 == 0 or d2 == 0:
            #The start or end being on the centroid means this is probably a tap, not a line of any sort
            return 0
        #From the law of cosines and https://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
        #This should be in radians, math.acos is in radians, according to the docs
        #value is capped at 1 because numerical imprecision was causing math domain errors by feeding
        #acos values like -1.00000000002, which is, technically, out of range
        capped = self.clamp((pow(d1, 2) + pow(d2, 2) - pow(d3, 2))/(2 * d1 * d2), -1, 1)
        return math.acos(capped)
        
    #Accepts a gesture, tries to recognize it, returns None if the gesture isn't recognized
    def recognize(self, gesture):
        #This isn't something you should just be calling, it's the superclass for the others
        return None

class TapRecognizer(GestureRecognizer):
    def __init__(self):
        super(TapRecognizer, self).__init__()
        self.name = "TapRecognizer"

    def recognize(self, gesture):
        if len(gesture) < 10:
            if self.getAvgCentroidDist(gesture) < 10:
                #Less than 10 events, close in position in space
                return True
        return False

class DoubleTapRecognizer(GestureRecognizer):
    def __init__(self):
        super(TapRecognizer, self).__init__()
        self.name = "DoubleTapRecognizer"

    def recognize(self, gesture):
    	return any([x.is_double_tap for x in gesture])

class TripleTapRecognizer(GestureRecognizer):
    def __init__(self):
        super(TapRecognizer, self).__init__()
        self.name = "TripleTapRecognizer"

    def recognize(self, gesture):
        return any([x.is_triple_tap for x in gesture])    	    	

class LineRecognizer(GestureRecognizer):
    def __init__(self):
        super(TapRecognizer, self).__init__()
        self.name = "LineRecognizer"

    def recognize(self, gesture):
        if self.getEndAngle(gesture) > 2.5:
            return True
        else
            return False

class ArcRecognizer(GestureRecognizer):
    def __init__(self):
        super(TapRecognizer, self).__init__()
        self.name = "ArcRecognizer"

    def recognize(self, gesture):
        endAngle = self.getEndAngle(gesture)
        if endAngle > 1 and endAngle < 2.5:
            return True
        return False

class CircleRecognizer(GestureRecognizer):
    def __init__(self):
        super(TapRecognizer, self).__init__()
        self.name = "CircleRecognizer"

    def recognize(self, gesture):
        #Emperically derived constant, is the angle around 
        #the centroid between the start and end points, in radians
        if self.getEndAngle < 1:
            return True


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
			print recognizer.name, output
		return output


def getRecognizer():
	recEng = RecognitionEngine()
	recEng.addRecognizer(TapRecognizer())
	recEng.addRecognizer(DoubleTapRecognizer())
	recEng.addRecognizer(TripleTapRecognizer())
	recEng.addRecognizer(LineRecognizer())
	recEng.addRecognizer(ArcRecognizer())
	recEng.addRecognizer(PolyRecognizer())
	#recEng.addRecognizer(GoRecognizer())
	return recEng