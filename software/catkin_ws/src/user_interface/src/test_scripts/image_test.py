#!/usr/bin/python

import kivy
kivy.require('1.9.1') # replace with your current kivy version !

from kivy.app import App
from kivy.uix.image import Image as UIXImage
from kivy.core.image import Image as CoreImage

from io import BytesIO

#Attempt to cure possibly creating the texture before the GL context exists
from kivy.base import EventLoop

class TestApp(App):

    def build(self):
        imgPath = "/usr/share/icons/Tango/16x16/actions/gnome-shutdown.png"

        #This works
        #return UIXImage(source=imgPath)

        EventLoop.ensure_window()
        
        #This works now
        data = BytesIO(open(imgPath, "rb").read())
        im = CoreImage(data, ext="png")
        #t = im.texture
        return UIXImage(texture=im.texture)

if __name__ == '__main__':
    TestApp().run()