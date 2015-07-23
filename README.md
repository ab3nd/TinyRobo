# TinyRobo
Hardware and software for a tiny robot platform based on the ESP-8266 WiFi modules and hacked children's toys. 

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

###The software

Once the hardware is complete, ROS modules will be written to control TinyRobos, and to emulate various effects, like directional communication (good for certain ant-like behaviors), unreliable inter-robot communication, and simulated stigmergy. 
These software modules will also be part of this repository, so the entire collection of materials will be able to be used by any researcher who wishes to use them. 

