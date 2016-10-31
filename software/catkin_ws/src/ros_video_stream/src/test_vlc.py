#!/usr/bin/python

import time

import sys 
sys.path.append('/home/ams/Projects/TinyRobo/software/catkin_ws/src/ros_video_stream/src/') 
import vlc

instance = vlc.Instance()

#Create a MediaPlayer with the default instance
player = instance.media_player_new()

#Load the media file
media = instance.media_new('rtsp://10.250.250.134/live.sdp')

#Add the media to the player
player.set_media(media)

#Play for 10 seconds then exit
player.play()
time.sleep(40)