#!/usr/bin/python

#Try to talk to the TinyRobo

import socket
import errno
import time

# Breadboard gets 192.168.2.187
# Unit on tank gets 192.168.2.167
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.2.167", 4321))
s.send('Q') 
#Pretty hackey, might not get the full thing in one go. 
received = s.recv(10)
print received

s.send(bytearray([ord('M'),0x1F, 0x02, 0x1F, 0x02]))

time.sleep(3)

s.send(bytearray([ord('M'),0x00, 0x00, 0x00, 0x00]))