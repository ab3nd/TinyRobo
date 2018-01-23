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

    def build(self):
        imgPath = "/usr/share/icons/Tango/16x16/actions/gnome-shutdown.png"

        #This works
        #return UIXImage(source=imgPath)

        EventLoop.ensure_window()
        
        #This works if the window is assured
        # data = BytesIO(open(imgPath, "rb").read())
        # im = CoreImage(data, ext="png")
        # return UIXImage(texture=im.texture)

        #This also works if the window is assured, but not from update_image
        image = PILImage.new('RGBA', size=(64, 64), color=(155, 255, 0))
        image.save(self.imageData, "PNG")
        self.imageData.seek(0)
            
        im = CoreImage(self.imageData, ext='png')
        
        self.image = UIXImage(texture=im.texture)

        Clock.schedule_interval(self.display_image, 4.0) #1.0 / 30.0)

        return self.image

    def display_image(self):
        try:
            image = PILImage.new('RGBA', size=(64, 64), color=(155, 55, 0))
            image.save(self.imageData, "PNG")
            self.imageData.seek(0)
                
            im = CoreImage(self.imageData, ext='png')

            with self.image:
                texture=im.texture
        except Exception as e:
            print e
            
    def update_image(self, imgMsg):

        #This is all apparently happening outside the GL context?
        
        image = PILImage.new('RGBA', size=(64, 64), color=(155, 5, 0))
        byteImgIO = BytesIO()
        image.save(byteImgIO, "PNG")
        byteImgIO.seek(0)
            
        im = CoreImage(byteImgIO, ext='png')
        
        # try:
        #     t = im.texture
        #     #self.image = UIXImage(texture=im.texture)
        # except Exception as inst:
        #     print inst

if __name__ == '__main__':
    StupidApp().run()

#Might come in handy later
# from threading import Thread
# spin_thread = Thread(target=lambda: rospy.spin())
# spin_thread.start()