#!/usr/bin/python

#An attempt at an utterly basic kivy program that creates an image from memory
#In this case, from an array of uint8 values that represent R,G,B values, so 
# [r,g,b,r,g,b,r,g....]

import kivy
kivy.require('1.9.1') # replace with your current kivy version !
from kivy.config import Config
#Don't resize the window
#This has to be before any other Kivy imports, or it fails quietly
Config.set('graphics', 'resizable', False)
#Config.set('graphics', 'fullscreen', 'fake')
Config.set('graphics', 'borderless', False)
from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.graphics.texture import Texture
from kivy.uix.image import Image as UIXImage

import numpy as np
from io import BytesIO  

def makeImage():
    width = 640
    height = 480
    px_count = width * height

    #Set up the colors 
    color_length = px_count/3
    reds = np.linspace(0,255, color_length, dtype=int)
    reds = np.append(reds[:-1], np.fliplr([reds])[0])
    reds = np.append(reds, np.zeros(2*color_length, dtype = int))
    greens = np.roll(reds, color_length)
    blues = np.roll(greens, color_length)
    #Combine them all
    colors = zip(reds, greens, blues)

    #Flatten the list
    colors = [element for x in colors for element in x]

    #Put it in a BytesIO
    buf = b''.join(map(chr, colors))
    byte_array = BytesIO(buf)
    byte_array.seek(0)

    #Convert it to a Kivy Texture
    tex = Texture.create(size=(width, height), colorfmt="rgb")
    tex.blit_buffer(byte_array.getvalue(), colorfmt='rgb', bufferfmt='ubyte')

    #Texture a widget with the texture and return that
    return UIXImage(texture=tex)

class StupidApp(App):

    def __init__(self, **kwargs):
        super(StupidApp, self).__init__(**kwargs)

    def build(self):
        main = GridLayout(cols=1)
        main.add_widget(makeImage())

    def on_pause(self):
        return True

    def on_stop(self):
        return True

if __name__ == '__main__':
    StupidApp().run()