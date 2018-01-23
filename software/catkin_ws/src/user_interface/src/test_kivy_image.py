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
from kivy.graphics.texture import Texture
from kivy.uix.image import Image as UIXImage
from kivy.core.image import Image as CoreImage
from kivy.uix.button import Button
from kivy.graphics import Rectangle
from array import array

#From the Kivy example this is based on, remove later
from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color, Rectangle
from random import random as r
from functools import partial


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

    def add_rects(self, label, wid, count, *largs):
        label.text = str(int(label.text) + count)
        with wid.canvas:
            for x in range(count):
                Color(r(), 1, 1, mode='hsv')
                Rectangle(pos=(r() * wid.width + wid.x,
                               r() * wid.height + wid.y), size=(20, 20))

    def double_rects(self, label, wid, *largs):
        count = int(label.text)
        self.add_rects(label, wid, count, *largs)

    def reset_rects(self, label, wid, *largs):
        label.text = '0'
        wid.canvas.clear()

    def build(self):
        wid = Widget()

        label = Label(text='0')

        btn_add100 = Button(text='+ 100 rects',
                            on_press=partial(self.add_rects, label, wid, 100))

        btn_add500 = Button(text='+ 500 rects',
                            on_press=partial(self.add_rects, label, wid, 500))

        btn_double = Button(text='x 2',
                            on_press=partial(self.double_rects, label, wid))

        btn_reset = Button(text='Reset',
                           on_press=partial(self.reset_rects, label, wid))

        layout = BoxLayout(size_hint=(1, None), height=50)
        layout.add_widget(btn_add100)
        layout.add_widget(btn_add500)
        layout.add_widget(btn_double)
        layout.add_widget(btn_reset)
        layout.add_widget(label)

        root = BoxLayout(orientation='vertical')
        root.add_widget(wid)
        root.add_widget(layout)

        return root
        
    def on_pause(self):
        return True

    def on_stop(self):
        return True

    def update_image(self, imgMsg):
        file = io.BytesIO()
        print "About to do the other thing!"
        image = PILImage.new('RGBA', size=(50, 50), color=(155, 255, 0))
        image.save(file, 'png')
        file.name = 'test.png'
        im = CoreImage(file, ext="png")

        #Put it into the canvas
        wid.add(Rectangle(size=(50,50), texture=im.texture))

        print "Did the thing!"


if __name__ == '__main__':
    StupidApp().run()