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

class SlideScreen(GridLayout):

	

	def __init__(self, **kwargs):
		super(SlideScreen, self).__init__(**kwargs)
		self.cols = 1
		self.nextButton = Button(text="Next")
		self.nextButton.bind(on_press=self.nextClickedCallback)
		self.bgImage = Image(source="./Swarm_Robot_Control_-_100_Robot_0028.png")
		self.add_widget(self.nextButton)
		self.add_widget(self.bgImage)


	def nextClickedCallback(self, value):
		print "Woooo"
		self.change_background("./Swarm_Robot_Control_-_100_Robot_0030.png")

	def change_background(self, new_path):
		self.bgImage.source = new_path
		self.bgImage.canvas.ask_update()
		
class MyApp(App):

    def build(self):
        return SlideScreen()

    def on_pause(self):
    	#This is where I'd save the log file
    	return True

    def on_stop(self):
    	#Also dump the log here
    	return True


if __name__ == '__main__':
    MyApp().run()