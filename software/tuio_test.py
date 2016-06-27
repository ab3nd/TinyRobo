#!/usr/bin/python

#Gets TUIO messages from the reactivision driver and uses them to tell you how 
#to rotate all the fiducials so they point the same way. This is stupid, but 
#it's a good start for remote controlling the robots. 

import tuio
import time
trkr = tuio.Tracking()

threshold = 5

try: 
	while True: 
		trkr.update() 
		for obj in trkr.objects(): 
			if obj.angle > threshold or obj.angle < 360 - threshold:
				if obj.angle < 180:
					print "Turn {0} left {1} degrees".format(obj.id, 360-obj.angle)
				else:
					print "Turn {0} right {1} degrees".format(obj.id, obj.angle)


except KeyboardInterrupt: 
	trkr.stop() 