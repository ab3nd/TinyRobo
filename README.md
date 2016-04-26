# TinyRobo
Hardware and software for a tiny robot platform based on the ESP-8266 WiFi modules and hacked children's toys. 

I've designed (but not tested yet) the second version of the control PCBs. 
You can buy them from [Dangerous Things](http://dirtypcbs.com/view.php?share=19993&accesskey=05f2c039a03e3f4ff6dc1a4ad2efa9ee) for like $1.50 each, but you'll have to provide all your own parts.

### The hardware
There are a lot of designs in the academic literature for small swarm robots. 
Generally speaking, smaller robots are better because the researchers can maintain a high ratio of space to robots in a smaller area. 
A high ratio of space to robots is good because it gives the robots more freedom of motion. 

Small swarm robots typically have three major electronic components:

1. One or more microprocessors
2. Relatively high-current drive electronics for the motors.
3. A communication system, to let swarm robots communicate either with each other or with a coordinating computer. 
4. (I said three, didn't I?) Power control electronics, including batteries. No one ever mentions this in papers, though. 

Hardware that provides at least two of these functions is commercially available. 
[Moteino](http://lowpowerlab.com/moteino/) provides wireless communication and a microcontroller, but doesn't have high-current drivers or power control except with a [daugtherboard](https://lowpowerlab.com/shop/powershield). Unfortunately, the antennas required for radio frequencies in the hundreds of Mhz are larger than the rest of the hardware. 
[Baby Orangutan](https://www.pololu.com/product/1220/specs) provides a microcontroller and two 1A H-bridge motor drivers, but no wireless communication. 

TinyRobo is intended to provide a single board that combines WiFi and a microcontroller, in the form of the ESP-8266-03, with a battery charger, voltage regulator, and dual H-bridge motor drivers.

In order to build small robots, many researchers develop their own chassis and drivetrains for the robots. 
This makes replication difficult, because the robots are rarely made commercially available. 
However, children's toys are commercially very available. 
Instead of using custom-made drivetrains, TinyRobo is intended to be a drop-in replacement for the controllers of children's toys. 
Combined with the right toy, the TinyRobo board will serve as a complete small robot that uses up to two motors for locomotion. This includes Ackerman-steering toy cars, differential-drive tanks, and holonomic-drive toy insect robots.

### Software for the ESP-8266 TinyRobo boards

As an initial cut at the hardware (very minimal functionality), my plan is this:

1. Arduino-compatible firmware on the ESP-8266, so that I can...
2. Use ROS/Arduino to recieve ROS messages and turn them into PWM signals for the two motor drivers, so that I can...
3. Drive the robot from RVIZ. 

Unfortunately, [ROSSerial](http://wiki.ros.org/rosserial_arduino/Tutorials) will let me do ROS messaging from an Arduino-like platform, but the Arduino development environment for the ESP-8266 doesn't have the serial-over-wifi interface that the ESP-8266 normally implements.
I assume that they would have to connect to something, using the libraries available on the Arduino environment for the ESP-8266, but there is no indication of what that thing is, or if it exists. 

Alternatively, I can use [ROSBridge](http://wiki.ros.org/rosbridge_suite) to do JSON-to-ROS serialization on the host computer, and have my robots publish JSON over TCP to ROSBridge.
This appears to be the winning plan, as the ESP-8266 can deal with the generation and parsing of the JSON messages, and ROSBridge provides something for it to connect to on the host computer. 

Attempting to install the WiFiWebServer demo on the stock ESP-8266 doesn't work. 
I currently have CH_PD pulled high and GPIO-15 pulled low, which allows it to communicate over the serial port. The error message is:
warning: espcomm_sync failed
error: espcomm_open failed

What I needed to do to have it work is boot the ESP-8266-03 with GPIO15 and GPIO0 pulled low, and CH_PD pulled high. 
Booting again with GPIO0 floating/not pulled low should (I hope) let me talk to it over serial AND have it running whatever firmware I put on it. 
Just for reference, the WiFiWebServer demo is 295k of 434k bytes, so getting a JSON lib and motor drive in should be cake. 

##### Basic hardware bringup for the ethernet module
1. Put it on a breadboard with a 1000uF cap across the power and BEEFY 3.3v power supply. I'm using a benchtop supply. 
2. 1k resistors, pull up pin CH_PD, pull down GPIO15 & GPIO0
3. Start the Arduino IDE, plug in a 3v3 FTDI cable
4. Start the Arduino Serial monitor at 9600 baud
5. See a lot of garbage, then [Vendor:www.ai-thinker.com Version:0.9.2.4] ready
6. Good to reflash, using the "program" button in the arduino IDE
7. If you want the program to run afterwards, you need to keep the pull up on CH_PD and the pull-down on GPIO15. 

##### Mobility platform current draw
Free run current was measured with the toy held so its moving parts didn't touch anything. 
This means that the free running current includes the current required to move the moving parts of the toy, but not to move the toy around. 
Stall current was measured by holding the toy's moving parts still. In the Hexbug, this causes some gear chatter, so the actual current draw when the motor is completely immobilized may be higher. 

Toy | Free current | Stall current
--- | --- | ---
Hexbug brand mini spider | 0.03A | 0.13A
Hexbug brand 6-legged insect | 0.06A | 0.25A
Miniature toy RC car |  0.21A | 0.8A
Miniature toy RC insect |  0.19A | 1.13A
Miniature toy RC vehicle |  0.37A | 0.8A
Miniature toy RC vehicle | 0.06A | 0.74A
Toy helicopter | 0.07A | 1.12A
Toy quadcopter | 0.74A | 1.99A

### E-Pucks

The initial development of the algorithims is going to be done on E-Pucks. 
So far, the only useful-looking software I've found for getting E-Pucks to talk to ROS is from these links. 
http://www.gctronic.com/doc/index.php/E-Puck
https://github.com/gctronic/epuck_driver
https://github.com/gctronic/epuck_driver_cpp

I have the unfortunate sense that this readme is going to turn into the notes file. 

The command to upload a file to a given robot is 
epuckuploadbt FILE ROBOT_ID
For the built version of the demoGCtronic firmware, the command looks like this (for robot 2151)

 ./epuckuploadbt ../../../../program/DemoGCtronic-complete/demoGCtronic.X/dist/default/production/demoGCtronic.X.production.hex 2151

In theory, I'll only have to flash each robot once, and then I can use ROS to control them from there on out. 

###The software

Once the hardware is complete, ROS modules will be written to control TinyRobos, and to emulate various effects, like directional communication (good for certain ant-like behaviors), unreliable inter-robot communication, and simulated stigmergy. 
These software modules will also be part of this repository, so the entire collection of materials will be able to be used by any researcher who wishes to use them. 

## OSHW
This project is open source in the sense that I legally permit people to do whatever they want with the source code, design drawings, etc.; and in the sense that I provide that information for them to do what they want. 
It is EXPLICITLY NOT COMPLIANT with the [OSHWA Certification](www.oshwa.org/2015/09/19/open-source-hardware-certification-version-1/). 
It never will be, because the only thing that certification adds to a project is increased liability to civil suits. 
