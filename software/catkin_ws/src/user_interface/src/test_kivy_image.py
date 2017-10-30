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

import numpy as np
from io import BytesIO  

import cv2

def makeImage():
    
    width = 32
    height = 32
    # create a 64x64 texture, defaults to rgb / ubyte
    texture = Texture.create(size=(width, height), colorfmt='rgb')

    # create 64x64 rgb tab, and fill with values from 0 to 255
    # we'll have a gradient from black to white
    size = width * height * 3
    buf = [int(x * 255 / size) for x in range(size)]

    # then, convert the array to a ubyte string
    arr = array('B', buf)
    # buf = b''.join(map(chr, buf))

    # then blit the buffer
    texture.blit_buffer(arr, colorfmt='rgb', bufferfmt='ubyte')
    # return UIXImage(texture=tex, size=(width, height))

    #return Button(text="WHY YOU NO RENDER")
    #tmp_img = np.reshape(tmp_img, (width, height, 3))
    #success, buf = cv2.imencode(".png", tmp_img)

    #data = BytesIO(buf)

    #import pdb; pdb.set_trace()
    
    #CoreImage rather than UIX image
    #im = CoreImage(data, ext=".png")
    
    #Create a UIX image from the core image
    #return UIXImage(texture = im.texture)

    #This route leads to a lot of segfaults
    #Convert it to a Kivy Texture
    # tex = Texture.create(size=(width, height), colorfmt="rgb", bufferfmt='ubyte')
    # tex.blit_buffer(byte_array.getvalue(), colorfmt='rgb', bufferfmt='ubyte')

    # #Texture a widget with the texture and return that
    # return UIXImage(texture=tex)
    img = UIXImage()
    with img.canvas:
        Rectangle(texture=texture, pos=(0, 0), size=(width*3, height*3))

    return img
    #return UIXImage(source="./32_33_34_35_36_28_29_31.png")

class StupidApp(App):

    def __init__(self, **kwargs):
        super(StupidApp, self).__init__(**kwargs)

    def build(self):
        return makeImage()
        #root.canvas.ask_update() 
        
    def on_pause(self):
        return True

    def on_stop(self):
        return True

if __name__ == '__main__':
    StupidApp().run()