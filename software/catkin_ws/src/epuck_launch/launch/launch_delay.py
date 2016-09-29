#!/usr/bin/python

import subprocess
import signal
import time
import sys

launchfiles=["epuck_2028.launch",
             "epuck_2039.launch",
             "epuck_2046.launch",
             "epuck_2087.launch",
             "epuck_2097.launch",
             "epuck_2099.launch",
             "epuck_2110.launch",
             "epuck_2117.launch",
             "epuck_2137.launch",
             "epuck_2151.launch",
             "epuck_2180.launch"]

procList = []

#Send everything a sigint, wait for them to die, then exit this process
def killEmAll(signal, frame):
	for proc in procList:
		proc.send_signal(signal)
	print "Waiting 10 sec for everything to shut down"
	time.sleep(10)
	sys.exit()

#When this process gets a sigint, send a signint to all the child processes
signal.signal(signal.SIGINT, killEmAll)

for launchfile in launchfiles:
	#Launch in a subprocess
	command = "roslaunch epuck_launch {0}".format(launchfile)
	#This is ok because launchfiles doesn't contain any malicous input. 
	#Don't put any in there, mkay?
	proc = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE) #, stderr=subprocess.PIPE)
	#add the process to the list
	procList.append(proc)
	#delay for a few seconds
	time.sleep(5)
	
print "Ctrl-C should kill everything"

while True:
	#for proc in procList:
	#	line = proc.stderr.readline()
	#	if line is not '':
	#		#Not EOF yet
	#		print line.rstrip()
	pass