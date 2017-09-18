#Gesture recognition classes and methods

 
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

    #Accepts a gesture, tries to recognize it, returns None if the gesture isn't recognized
    def recognize(self, gesture):
        #This isn't something you should just be calling, it's the superclass for the others
        return None

class TapRecognizer(GestureRecognizer):
    def recognize(self, gesture):
    	import pdb; pdb.set_trace()
    	return (len(gesture.events) <= 10) and (self.avgCenterDist() < 20)

class DoubleTapRecognizer(GestureRecognizer):
    def __init__(self):
        #Gestures, stored by UID
        self.pastGestures = {}

    def recognize(self, gesture):
    	return None

class TripleTapRecognizer(GestureRecognizer):
    def __init__(self):
        #Gestures, stored by UID
        self.pastGestures = {}

    def recognize(self, gesture):
    	return None    	

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