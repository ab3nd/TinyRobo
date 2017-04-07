#!/usr/bin/python

#Display JPGs and log user interactions 

import kivy
kivy.require('1.9.1') # replace with your current kivy version !

from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.properties import ObjectProperty
from kivy.uix.image import Image
from kivy.uix.button import Button
from kivy.config import Config
from kivy.uix.widget import Widget
from kivy.graphics import Color, Ellipse, Line

class TouchRecorder():
    def __init__(self):
        #Create a file name to log to
        #Create a start log entry
        pass

    #Timestamps are in unix time, seconds since the epoch, down to 10ths of a second. 
    def log_event(self, event):
        print event.uid, event.time_start, event.time_update, event.time_end, event.x, event.y, event.shape

#Widget that records all finger motion events on it
#TODO move the background image stuff to this, rather 
#than having it be in the layout
class FingerDrawer(Widget):

    def __init__(self, **kwargs):
        super(FingerDrawer, self).__init__(**kwargs)
        self.d = 8.
        self.tr = TouchRecorder()

    def on_touch_down(self, touch):
        self.tr.log_event(touch)
        with self.canvas:
            Color(0.5, 0.8, 0)
            Ellipse(pos=(touch.x - self.d, touch.y - self.d), size=(self.d * 2, self.d * 2))
            touch.ud['line'] = Line(points=(touch.x, touch.y), width=self.d)

    def on_touch_up(self, touch):
        self.tr.log_event(touch)
        with self.canvas:
            Ellipse(pos=(touch.x - self.d, touch.y - self.d), size=(self.d * 2, self.d * 2))

    def on_touch_move(self, touch):
        self.tr.log_event(touch)
        touch.ud['line'].points += [touch.x, touch.y]

    def clean_up(self):
        self.canvas.clear()


class SlideScreen(FloatLayout):  

    def __init__(self, **kwargs):
        super(SlideScreen, self).__init__(**kwargs)
        self.cols = 1
        
        #Get the app configuration and count the total slides
        self.cfg = Config.get_configparser('app')
        self.slideCount = len(self.cfg.items("Files"))
        #We're looking at the first slide
        self.slideIndex = 1
        
        #Background image of the task in question
        self.bgImage = Image(source = self.cfg.get("Files", str(self.slideIndex)))
        self.add_widget(self.bgImage)
        
        #Widget that records finger motions, defaults to being as big as the screen
        self.fr = FingerDrawer()
        self.add_widget(self.fr)

        #Small button for advancing the slide
        self.nextButton = Button(text="Next", size_hint=(0.07, 0.07))
        self.nextButton.bind(on_press = self.nextClickedCallback)
        self.add_widget(self.nextButton)
        

    def nextClickedCallback(self, value):
        #Increment the slide index and wrap if needed
        self.slideIndex += 1
        if self.slideIndex > self.slideCount:
            self.slideIndex = 1
        self.change_background(self.cfg.get("Files", str(self.slideIndex)))
        self.fr.clean_up()

    def change_background(self, new_path):
        self.bgImage.source = new_path
        self.bgImage.canvas.ask_update()
        
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
        cfg = self.config
        return SlideScreen()

    def on_pause(self):
        #This is where I'd save the log file
        return True

    def on_stop(self):
        #Also dump the log here
        return True


if __name__ == '__main__':
    UITestApp().run()