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
	if not ip.startswith("127"):
		scan_ips.append(ip)

for ip in scan_ips:
	for ii in range(255): #Scan the whole /8
		targetIP = ".".join(ip.split(".")[0:3])
		targetIP = targetIP + ".{0}".format(ii)
		#Try to connect on port 4321. If no connection,
		#then it's probably not a TinyRobo
		try:
			print targetIP
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.connect((targetIP, 4321))
			s.send('Q') 
			#Pretty hackey, might not get the full thing in one go. 
			received = s.recv(10)
			if received.startswith('TinyRobo'):
				print "Found TinyRobo at {0}".format(targetIP)
		except socket.error as sockErr:
			if sockErr.errno == errno.ECONNREFUSED:
				pass
				#print "Connection refused on {0}".format(targetIP)
			elif sockErr.errno == errno.ENETUNREACH:
				pass
				#print "{0} is unreachable.".format(targetIP)
			elif sockErr.errno == 113: #errno.EHOSTUNREACH:
				pass
				#print "How is this different from ENETUNREACH?"	
			else:
				raise sockErr
		finally:
			#s.shutdown(socket.SHUT_RDWR)
			s.close()

			#Otherwise, eat it and continue


