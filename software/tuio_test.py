#!/usr/bin/python

#Gets TUIO messages from the reactivision driver and uses them to tell you how 
#to rotate all the fiducials so they point the same way. This is stupid, but 
#it's a good start for remote controlling the robots. 

import tuio
trkr = tuio.Tracking()

try: 
	while True: 
		trkr.update() 
		for obj in trkr.objects(): 
			print obj 
except KeyboardInterrupt: 
	trkr.stop() 