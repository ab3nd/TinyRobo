#!/bin/sh

#Pair all the pucks, on two different interfaces

bluez-simple-agent hci0 10:00:E8:AD:77:6E
bluez-test-device trusted 10:00:E8:AD:77:6E yes

bluez-simple-agent hci0 10:00:E8:AD:76:FD
bluez-test-device trusted 10:00:E8:AD:76:FD yes

bluez-simple-agent hci0 10:00:E8:AD:77:B4
bluez-test-device trusted 10:00:E8:AD:77:B4 yes

bluez-simple-agent hci0 10:00:E8:AD:77:AE
bluez-test-device trusted 10:00:E8:AD:77:AE yes

bluez-simple-agent hci0 10:00:E8:AD:77:C2
bluez-test-device trusted 10:00:E8:AD:77:C2 yes

bluez-simple-agent hci0 10:00:E8:AD:79:B3	
bluez-test-device trusted 10:00:E8:AD:79:B3 yes

bluez-simple-agent hci1 10:00:E8:AD:5B:AA
bluez-test-device trusted 10:00:E8:AD:5B:AA yes

bluez-simple-agent hci1 10:00:E8:AD:77:F5
bluez-test-device trusted 10:00:E8:AD:77:F5 yes

bluez-simple-agent hci1 10:00:E8:AD:5B:D7
bluez-test-device trusted 10:00:E8:AD:5B:D7 yes

bluez-simple-agent hci1 10:00:E8:AD:77:ED
bluez-test-device trusted 10:00:E8:AD:77:ED yes

bluez-simple-agent hci1 10:00:E8:AD:5B:B5
bluez-test-device trusted 10:00:E8:AD:5B:B5 yes
