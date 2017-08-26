#!/usr/bin/python

#Display JPGs and log user interactions 

import kivy
kivy.require('1.9.1') # replace with your current kivy version !
from kivy.config import Config
#Don't resize the window
#This has to be before any other Kivy imports, or it fails quietly
#Config.set('graphics', 'resizable', False)
#Config.set('graphics', 'fullscreen', 'fake')
Config.set('graphics', 'borderless', False)
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.properties import ObjectProperty
from kivy.uix.image import Image as kvImage
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.graphics import Color, Ellipse, Line
#For keyboard listener and automatic sizing to slides
from kivy.core.window import Window

import pickle
import datetime
import time

#Just for getting the size of images
import Image 

import sys

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

        #Get the subject id and condition from the app config
        self.cfg = Config.get_configparser('app')
        subject = self.cfg.get('Subject', 'id')
        condition = self.cfg.get('Condition', 'type')

        #Create a file name to log to
        self.fName = time.strftime("%d-%m-%y_%H:%M:%S_s{0}_c{1}".format(subject, condition.split("_")[1])) + ".pickle"
        if prefix is not None:
            self.fName = prefix + fName

        self.outfile = open(self.fName, 'w')

    #Timestamps are in unix time, seconds since the epoch, down to 10ths of a second. 
    def log_touch_event(self, event):
        print event.x, event.y
        
        if event.shape is not None:
            shape_width = event.shape.width
            shape_height = event.shape.height
        else:
            shape_width = shape_height = None

        event = {"time": event.time_update,
                 "uid": event.uid,
                 "start_time" : event.time_start,
                 "end_time" : event.time_end,
                 "event_x" : event.x,
                 "event_y" : event.y,
                 "update_time" : event.time_update,
                 #"shape" : event.shape} Shapes are not picklable
                 #but as of kivy 1.10, they're only ever rectangular...
                 "shape" : {"width" : shape_width, "height" : shape_height}}
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

class MultiImage(kvImage):
    def __init__(self, **kwargs):
        super(MultiImage, self).__init__(**kwargs)
        
        #Get the app configuration and count the total slides
        self.cfg = Config.get_configparser('app')
        
        #Get the configuration section, and from that, the images
        self.cfg = Config.get_configparser('app')
        self.condition = self.cfg.get('Condition', 'type')

        self.slideCount = len(self.cfg.items(self.condition))

        #We're looking at the first slide
        self.slideIndex = 1

        #Record touch events
        self.tr = TouchRecorder()

        #Set ourselves up with the inital image
        self.source = self.cfg.get(self.condition, str(self.slideIndex))
        
        #Resize to match the initial image. They're all the same size, so this is legit
        img = Image.open(self.source)
        width, height = img.size
        Window.size = (width, height)
        
        #Log that we loaded it and refresh the view
        self.tr.log_meta_event("Loaded {0}".format(self.source))
        self.canvas.ask_update()

    def nextSlide(self):
        #Increment the slide index and wrap if needed
        self.slideIndex += 1
        if self.slideIndex >= self.slideCount:
            self.slideIndex = 1
        #New image for the background
        self.source = self.cfg.get(self.condition, str(self.slideIndex))

        #Log what file was loaded
        self.tr.log_meta_event("Loaded {0}".format(self.source))

        #Widget is the same size as the image
        self.canvas.ask_update()
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

    def __init__(self, **kwargs):
        super(UITestApp, self).__init__(**kwargs)
        self.condition = condition
        self.subject = subject

    def build_config(self, config):
        #If you don't set any defaults, Kivy won't load your config at all 
        config.setdefaults('Condition', {'type': 'files_unknown'})
        config.setdefaults('Subject', {'id': '--undef--'})
        
        #Using the parameters, set the configuration section
        if self.condition == '1':
            config.set('Condition', 'type', 'files_single')
        if self.condition == '10':
            config.set('Condition', 'type', 'files_10')
        if self.condition == '100':
            config.set('Condition', 'type', 'files_100')
        if self.condition == '1000':
            config.set('Condition', 'type', 'files_1000')
        if self.condition == 'X':
            config.set('Condition', 'type', 'files_unknown')

        config.set("Subject", 'id', self.subject)
        #config.set('graphics', 'fullscreen', 'auto')

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
    conditions = ['1', '10', '100', '1000','X']
    parser = argparse.ArgumentParser(description = "Display and log contact points for PhD experiment")
    parser.add_argument('-i', nargs='?', default=0, type=int, required=True, help='Numerical subject identifier')
    helptext = "One of " + ", ".join(conditions)
    parser.add_argument('-c', nargs='?', required=True, help=helptext)
    args = parser.parse_args()

    condition = 0
    subject = 0

    #Check if the condition argument makes sense. 
    if vars(args)['c'] not in conditions:
        print "Condition must be one of " + ", ".join(conditions)
        sys.exit(-1)
    else:
        condition = vars(args)['c']
    
    subject = vars(args)['i']

    UITestApp(condition = condition, subject = subject).run()
