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

I've recently switched to using http://wiki.ros.org/aruco_detect (although https://github.com/pal-robotics/aruco_ros can probably do a lot of the same things). When I remove my attempt at making a simulator and my modded version of APRIL tags, the dependencies of this section will change a lot. 



