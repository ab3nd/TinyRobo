#!/usr/bin/python

#Display JPGs and log user interactions 
#This is terrible (kinda globals, and sys.argv parsing by hand to dodge kivy)
# import sys
# condition = 0
# participant_id = 0
# for arg_index in range(len(sys.argv)):
#     arg = sys.argv[arg_index]
#     print arg_index, arg
#     if arg == '-c':
#         condition = int(sys.argv[arg_index])
#         sys.argv.remove(arg)
        
#     if arg == '-i':
#         sys.argv.remove(arg)
#         participant_id = i

# print condition, participant_id
#The above doesn't work due to bad array manipulation, but 
#leaving them in place breaks kivy. Start at the end of the array, and walk back?


import kivy
kivy.require('1.9.1') # replace with your current kivy version !
from kivy.config import Config
#Don't resize the window
#This has to be before any other Kivy imports, or it fails quietly
Config.set('graphics', 'resizable', False)

from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.properties import ObjectProperty
from kivy.uix.image import Image
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.graphics import Color, Ellipse, Line
#For keyboard listener
from kivy.core.window import Window

import pickle
import datetime
import time

#Singleton-ifies things, so you only get one instance
def singleton(cls):
    instances = {}
    def getinstance():
        if cls not in instances:
            instances[cls] = cls()
        return instances[cls]
    return getinstance

@singleton
class TouchRecorder():
    def __init__(self, prefix=None):
        #Create a file name to log to
        self.fName = time.strftime("%d-%m-%y_%H:%M:%S") + ".pickle"
        if prefix is not None:
            self.fName = prefix + fName

        self.outfile = open(self.fName, 'w')

    #Timestamps are in unix time, seconds since the epoch, down to 10ths of a second. 
    def log_touch_event(self, event):
        event = {"time": event.time_update,
                 "uid": event.uid,
                 "start_time" : event.time_start,
                 "end_time" : event.time_end,
                 "event_x" : event.x,
                 "event_y" : event.y,
                 "update_time" : event.time_update,
                 "shape" : event.shape}
        pickle.dump(event, self.outfile)
        #Paranoia
        self.outfile.flush()

    def log_meta_event(self, desc):
        event = {"time": time.time(),
                 "desc": desc}
        pickle.dump(event, self.outfile)
        #Paranoia
        self.outfile.flush()

#Widget that records all finger motion events on it
#TODO move the background image stuff to this, rather 
#than having it be in the layout
class FingerDrawer(Widget):

    def __init__(self, **kwargs):
        super(FingerDrawer, self).__init__(**kwargs)
        self.d = 8.
        self.tr = TouchRecorder()
        Window.bind(size=self.reSize)

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)
            with self.canvas:
                Color(0.5, 0.8, 0)
                Ellipse(pos=(touch.x - self.d, touch.y - self.d), size=(self.d * 2, self.d * 2))
                touch.ud['line'] = Line(points=(touch.x, touch.y), width=self.d)

    def on_touch_up(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)
            with self.canvas:
                Ellipse(pos=(touch.x - self.d, touch.y - self.d), size=(self.d * 2, self.d * 2))

    def on_touch_move(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)
            touch.ud['line'].points += [touch.x, touch.y]

    def reSize(self, width, height):
        self.width = self.parent.ids["slide_show"].texture.width
        self.height = self.parent.ids["slide_show"].texture.height
        print self.size

    def clean_up(self):
        self.canvas.clear()

class MultiImage(Image):
    def __init__(self, **kwargs):
        super(MultiImage, self).__init__(**kwargs)
        
        #Get the app configuration and count the total slides
        self.cfg = Config.get_configparser('app')
        self.slideCount = len(self.cfg.items("Files"))
        #We're looking at the first slide
        self.slideIndex = 1

        #Set ourselves up with the inital image
        self.source = self.cfg.get("Files", str(self.slideIndex))

        #Record touch events
        self.tr = TouchRecorder()

    def nextSlide(self):
        #Increment the slide index and wrap if needed
        self.slideIndex += 1
        if self.slideIndex > self.slideCount:
            self.slideIndex = 1
        #New image for the background
        self.source = self.cfg.get("Files", str(self.slideIndex))
        #Widget is the same size as the image
        self.canvas.ask_update()

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)

    def on_touch_up(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)

    def on_touch_move(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)

#A lot of this was copied from the kivy example at 
#https://kivy.org/docs/api-kivy.core.window.html
class KeyboardListener(Widget):
    def __init__(self, **kwargs):
        super(KeyboardListener, self).__init__(**kwargs)
        self._kbrd = Window.request_keyboard(self._keyboard_closed, self, 'text')
        if self._kbrd.widget:
            # If it exists, this widget is a VKeyboard object which you can use
            # to change the keyboard layout.
            pass
        self._kbrd.bind(on_key_down=self._on_keyboard_down)
        self.tr = TouchRecorder()

    
    def _keyboard_closed(self):
        self._kbrd.unbind(on_key_down=self._on_keyboard_down)
        self._kbrd = None

    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        # Keycode is composed of an integer + a string
        if keycode[1] == 'n':
            self.tr.log_meta_event("Advanced slide")
            #TODO THIS FAILS IF THE WIDGET TREE CHANGES
            self.parent.ids["slide_show"].nextSlide()
            #self.parent.ids["finger_draw"].clean_up()
        if keycode[1] == 'c':
            self.tr.log_meta_event("Cleared screen for user")
            #self.parent.ids["finger_draw"].clean_up()
        if keycode[1] == 'q':
            self.tr.log_meta_event("Quit experiment")
            keyboard.release()
            App.get_running_app().stop()
            #self.parent.ids["finger_draw"].clean_up()

        # Return True to accept the key. Otherwise, it will be used by
        # the system.
        return True

class UITestApp(App):
    #Load the ini file from the working directory instead of who-knows-where
    #def get_application_config(self):
    #    return super(UITestApp, self).get_application_config('./%(appname).ini')

    def build_config(self, config):
        #If you don't set any defaults, Kivy won't load your config at all 
        config.setdefaults('Files', {'1': 'value1'})
        #Kivy module that draws a ring around touches, unfortunately uses a big ole 
        #PNG file that is white, and so doesn't show up on white backgrounds
        #Config.set('modules', 'touchring', '')

    def build(self):
        pass

    def on_pause(self):
        #This is where I'd save the log file
        return True

    def on_stop(self):
        #Also dump the log here
        return True


if __name__ == '__main__':
    #Kivy apparently ignores things after --
    import argparse
    conditions = ['1','10','100','1000','X']
    parser = argparse.ArgumentParser(description = "Display and log contact points for PhD experiment")
    parser.add_argument('-i', nargs='?', default=0, type=int, help='Numerical subject identifier')
    helptext = "One of " + ", ".join(conditions)
    parser.add_argument('-c', nargs='?', default=0, type=int, help=helptext)
    args = parser.parse_args()
    print args

    UITestApp().run()