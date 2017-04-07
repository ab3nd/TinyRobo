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


#Widget that records all finger motion events on it
#TODO move the background image stuff to this, rather 
#than having it be in the layout
class FingerRecorder(Widget):
    def on_touch_down(self, touch):
        with self.canvas:
            Color(0.5, 0.8, 0)
            #d = 30.
            #Ellipse(pos=(touch.x -d/2, touch.y-d/2), size=(d,d))
            touch.ud['line'] = Line(points=(touch.x, touch.y))

    def on_touch_up(self, touch):
        print(touch)

    def on_touch_move(self, touch):
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
        self.fr = FingerRecorder()
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