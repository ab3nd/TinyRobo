#!/usr/bin/python

#Display JPGs and log user interactions 

import kivy
kivy.require('1.9.1') # replace with your current kivy version !

from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.properties import ObjectProperty
from kivy.uix.image import Image
from kivy.uix.button import Button
from kivy.config import Config

class SlideScreen(GridLayout):	

	def __init__(self, **kwargs):
		super(SlideScreen, self).__init__(**kwargs)
		self.cols = 1
		self.nextButton = Button(text="Next")
		
		#Get the app configuration and count the total slides
		self.cfg = Config.get_configparser('app')
		import pdb; pdb.set_trace()
		self.slideCount = len(self.cfg.items("Files"))
		#We're looking at the first slide
		self.slideIndex = 1
		
		self.nextButton.bind(on_press=self.nextClickedCallback)
		self.bgImage = Image(self.cfg.get("Files", str(slideIndex)))

		self.add_widget(self.nextButton)
		self.add_widget(self.bgImage)


	def nextClickedCallback(self, value):
		#Increment the slide index and wrap if needed
		self.slideIndex += 1
		if self.slideIndex > self.slideCount:
			self.slideIndex = 1
		self.change_background(self.cfg.get("Files", str(slideIndex)))

	def change_background(self, new_path):
		self.bgImage.source = new_path
		self.bgImage.canvas.ask_update()
		
class UITestApp(App):

    def build_config(self, config):
    	#config.setdefaults()
    	pass

    def build(self):
    	return SlideScreen()

    def on_pause(self):
    	#This is where I'd save the log file
    	return True

    def on_stop(self):
    	#Also dump the log here
    	return True


if __name__ == '__main__':
    UITestApp().run()