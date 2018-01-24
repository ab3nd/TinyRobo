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

#For test, remove later
from PIL import Image as PILImage

import numpy as np
from io import BytesIO  
import cv2

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image

class StupidApp(App):

    def __init__(self, **kwargs):
        super(StupidApp, self).__init__(**kwargs)

        #intialize ROS and subscribe to an image topic
        topic = "/overhead_cam/image_rect_color"
        rospy.init_node('kivy_img_mauler')
        #We can get away with not calling rospy.Spin() because Kivy keeps it running
        self.sub = rospy.Subscriber(topic, Image, self.update_image)
        
        self.imageData = BytesIO()

        Clock.schedule_interval(self.display_image, 4.0) #1.0 / 30.0)

    def build(self):

        EventLoop.ensure_window()

        #This also works if the window is assured, but not from update_image
        image = PILImage.new('RGBA', size=(64, 64), color=(155, 255, 0))
        image.save(self.imageData, "PNG")
        self.imageData.seek(0)
            
        im = CoreImage(self.imageData, ext='png')
        
        try:
            #Set up the layout, and add a widget to it
            self.layout = FloatLayout()
            self.widget = Widget()
            self.layout.add_widget(self.widget)

            self.widget.canvas.clear()
            with self.widget.canvas:
                Rectangle(texture=im.texture)
        except Exception as e:
            print e

        return self.layout

    def display_image(self, dt):
        print "CALLED"
        try:
            image = PILImage.new('RGBA', size=(64, 64), color=(155, 5, 100))
            image.save(self.imageData, "PNG")
            self.imageData.seek(0)
            print "Did the first thing"
            im = CoreImage(self.imageData, ext='png')
            print "Did the second thing"
            self.widget.canvas.clear()
            with self.widget.canvas:
                Rectangle(texture = im.texture)

            print "Did the thing"
        except Exception as e:
            print e

        return True

    def update_image(self, imgMsg):
        #This is all apparently happening outside the GL context?
        
        # image = PILImage.new('RGBA', size=(64, 64), color=(155, 5, 0))
        # byteImgIO = BytesIO()
        # image.save(byteImgIO, "PNG")
        # byteImgIO.seek(0)
            
        # im = CoreImage(byteImgIO, ext='png')
        
        # try:
        #     t = im.texture
        #     #self.image = UIXImage(texture=im.texture)
        # except Exception as inst:
        #     print inst
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