#!/usr/bin/python

#A Kivy app with a ROS camera feed as the background
import kivy
kivy.require('1.9.1')

#Don't resize the window
#This has to be before any other Kivy imports, or it fails quietly
from kivy.config import Config
Config.set('graphics', 'resizable', False)
#Config.set('graphics', 'fullscreen', "auto") #Setting this to 1 leaves screen in wrong resolution
#Config.set('graphics', 'borderless', False)
from kivy.app import App

from kivy.core.window import Window
from kivy.uix.widget import Widget
from kivy.uix.image import Image as kvImage
from kivy.graphics.texture import Texture
from kivy.uix.camera import Camera
from kivy.graphics import Color, Ellipse, Line

#At the moment, just listens for the quit command
class KeyboardListener(Widget):
    def __init__(self, **kwargs):
        super(KeyboardListener, self).__init__(**kwargs)
        self._kbrd = Window.request_keyboard(self._keyboard_closed, self, 'text')
        if self._kbrd.widget:
            # If it exists, this widget is a VKeyboard object which you can use
            # to change the keyboard layout.
            pass
        self._kbrd.bind(on_key_down=self._on_keyboard_down)
    
    def _keyboard_closed(self):
        self._kbrd.unbind(on_key_down=self._on_keyboard_down)
        self._kbrd = None

    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        if keycode[1] == 'q':
            keyboard.release()
            App.get_running_app().stop()

        # Return True to accept the key. Otherwise, it will be used by
        # the system.
        return True

#Widget that records all finger motion events on it
class FingerWatcher(Widget):

    def __init__(self, **kwargs):
        super(FingerWatcher, self).__init__(**kwargs)
        self.d = 8.
        Window.bind(size=self.reSize)

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            with self.canvas:
                Color(0.5, 0.8, 0)
                Ellipse(pos=(touch.x - self.d, touch.y - self.d), size=(self.d * 2, self.d * 2))
                touch.ud['line'] = Line(points=(touch.x, touch.y), width=self.d)

    def on_touch_up(self, touch):
        if self.collide_point(*touch.pos):
            with self.canvas:
                Ellipse(pos=(touch.x - self.d, touch.y - self.d), size=(self.d * 2, self.d * 2))

    def on_touch_move(self, touch):
        if self.collide_point(*touch.pos):
            touch.ud['line'].points += [touch.x, touch.y]

    def reSize(self, width, height):
        self.width = self.parent.ids["slide_show"].texture.width
        self.height = self.parent.ids["slide_show"].texture.height
        print self.size

    def clean_up(self):
        self.canvas.clear()

class RobotUIMainApp(App):
    def __init__(self, **kwargs):
        super(RobotUIMainApp, self).__init__(**kwargs)
        
    def on_pause(self):
        #Not sure this should do anything
        return True

    def on_stop(self):
        #Unsubscribe from ROS messages
        return True

if __name__ == '__main__':
    RobotUIMainApp().run()
    