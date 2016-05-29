#!/usr/bin/python

#Try to talk to the TinyRobo

import socket
import errno

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.2.158", 4321))
s.send('Q') 
#Pretty hackey, might not get the full thing in one go. 
received = s.recv(10)
print received