#!/usr/bin/python

#An attempt at an utterly basic kivy program that creates an image from memory
#In this case, from an array of uint8 values that represent R,G,B values, so 
# [r,g,b,r,g,b,r,g....]

import kivy
kivy.require('1.9.1') # replace with your current kivy version !
from kivy.config import Config
#Don't resize the window
#This has to be before any other Kivy imports, or it fails quietly
#Config.set('graphics', 'resizable', False)
#Config.set('graphics', 'fullscreen', 'fake')
#Config.set('graphics', 'borderless', False)
from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.image import Image as UIXImage
from kivy.core.image import Image as CoreImage
from kivy.graphics import Rectangle
from kivy.clock import Clock
from kivy.base import EventLoop
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.widget import Widget
from kivy.core.image.img_pygame import ImageLoaderPygame
#For test, remove later
from PIL import Image as PILImage

import numpy as np
from io import BytesIO  
from StringIO import StringIO
import cv2

import sys

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import traceback

class StupidApp(App):

    def __init__(self, **kwargs):
        super(StupidApp, self).__init__(**kwargs)

        #intialize ROS and subscribe to an image topic
        topic = "/overhead_cam/image_rect_color/compressed"
        rospy.init_node('kivy_img_mauler')
        #We can get away with not calling rospy.Spin() because Kivy keeps it running
        self.sub = rospy.Subscriber(topic, CompressedImage, self.update_image)
        
        self.rosImgData = None

    def build(self):

        EventLoop.ensure_window()

        Clock.schedule_interval(self.display_image, 3.0) #1.0 / 30.0)
        
        #This also works if the window is assured, but not from update_image
        #image = PILImage.new('RGBA', size=(64, 64), color=(5, 55, 0))
        #image.save(self.imageData, "PNG")
        #self.imageData.seek(0)
            
        #im = CoreImage(self.imageData, ext='png')
        
        try:
            #Set up the layout, and add a widget to it
            self.layout = FloatLayout()
            self.widget = Widget()
            self.layout.add_widget(self.widget)

            #self.widget.canvas.clear()
            #with self.widget.canvas:
            #    Rectangle(texture=im.texture)
        except Exception as e:
            print e
            print (traceback.format_exc(sys.exc_info()))

        return self.layout

    def display_image(self, dt):
        print "CALLED"
        try:
            imgBuf = StringIO()
            imgBuf.write(self.rosImgData)
            imgBuf.seek(0)
            imData = ImageLoaderPygame(imgBuf).texture

            #Maybe do some size fix?
            
            self.widget.canvas.clear()
            with self.widget.canvas:
                Rectangle(texture=imdata)

        except Exception as e:
            print e
            print (traceback.format_exc(sys.exc_info()))

        return True

    def update_image(self, imgMsg):
        self.rosImgData = imgMsg.data
        print "Callback"
        return True

    def on_pause(self):
        #Not sure this should do anything
        return True

    def on_stop(self):
        #Unsubscribe from ROS messages
        return True

if __name__ == '__main__':
    StupidApp().run()

#Might come in handy later
# from threading import Thread
# spin_thread = Thread(target=lambda: rospy.spin())
# spin_thread.start()