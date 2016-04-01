#!/usr/bin/python

# Discovery script, finds the ESP8266 wifi modules on a network
# Eventually, this will be used to generate configurations
# for ROS to talk to a bunch of ESP8266 robots. 

import netifaces as ni
import socket
import errno

# Get a list of interfaces and their IP address. 
# netifaces may not work on Windows. Get a real computer. 
scan_ips = []
for interface in ni.interfaces():
	ip = ni.ifaddresses(interface)[2][0]['addr']
	print interface, ip
	if not (ip.startswith("127") or ip.startswith("10")):
		scan_ips.append(ip)

for ip in scan_ips:
	for ii in range(255): #Scan the whole /8
		targetIP = ".".join(ip.split(".")[0:3])
		targetIP = targetIP + ".{0}".format(ii)
		#Try to connect on port 4321. If no connection,
		#then it's probably not a TinyRobo
		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.connect((ip, 4321))
			print ip
		except socket.error as sockErr:
			if sockErr.errno != errno.ECONNREFUSED:
				raise sockErr
			#Otherwise, eat it and continue


