# TinyRobo
Hardware and software for a tiny robot platform based on the ESP-8266 WiFi modules and hacked children's toys. 

## Installation 

git clone https://github.com/ab3nd/TinyRobo

cd TinyRobo/software/catkin_ws/

sudo apt-get install libmagick++-dev

sudo ln -s /usr/include/ImageMagick-6/ /usr/include/ImageMagick

sudo apt-get install libmagickcore-dev

sudo apt-get install libmagickwand-6.q16-2

sudo apt-cache search libMagick++

sudo apt-get install libmagick++-6.q16-5v5

sudo apt-get install libmagick++-6.q16-dev  libmagickcore-dev libmagick++-dev

sudo ln -s /usr/lib/x86_64-linux-gnu/libMagickCore-6.Q16.so /usr/lib/x86_64-linux-gnu/libMagickCore.so

sudo ln -s /usr/lib/x86_64-linux-gnu/libMagick++-6.Q16.so /usr/lib/x86_64-linux-gnu/libMagick++.so

You'll also need the [Apriltags_ros](https://github.com/RIVeR-Lab/apriltags_ros) and [video_stream_opencv](https://github.com/ros-drivers/video_stream_opencv) libraries before you use catkin_make.

I've recently switched to using http://wiki.ros.org/aruco_detect (although https://github.com/pal-robotics/aruco_ros can probably do a lot of the same things). When I remove my attempt at making a simulator and my modded version of APRIL tags, the dependencies of this section will change a lot. 

## OSHW
This project is open source in the sense that I legally permit people to do whatever they want with the source code, design drawings, etc.; and in the sense that I provide that information for them to do what they want. 
It does not comply with anyone's idea of open source but my own, but given that I'm counting on github to be my last-resort backup in case of my University suffering a total existance failure, it behooves me to put everything I'd need to duplicate the work here. 
Let me know if I forgot to check something in. 


