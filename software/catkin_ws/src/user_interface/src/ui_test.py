#!/usr/bin/python

#Display JPGs and log user interactions 

import kivy
kivy.require('1.9.1') # replace with your current kivy version !
from kivy.config import Config
#Don't resize the window
#This has to be before any other Kivy imports, or it fails quietly
#Config.set('graphics', 'resizable', False)
#Config.set('graphics', 'fullscreen', 'fake')
#Config.set('graphics', 'borderless', False)
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
from kivy.logger import Logger

import pickle
import datetime
import time

#Just for getting the size of images
import Image 

import sys

#Let's make everything emit ROS messages!
import rospy
from std_msgs.msg import Bool                   #Whether the "interesting" key was pressed
from geometry_msgs.msg import PointStamped      #The point the user is touching
from sensor_msgs.msg import Image as ROSImgMsg  #The slide the user is viewing
from std_msgs.msg import String as ROSStrMsg    #Metaevents like quitting the app

#Singleton-ifies things, so you only get one instance
def singleton(cls):
    instances = {}
    def getinstance():
        if cls not in instances:
            print "NewInstance of {0}".format(cls)
            instances[cls] = cls()
        return instances[cls]
    return getinstance

#The decorator changes the type of TouchRecorder  
#into a function, which can't be subclassed, causing the error 
# TypeError: Error when calling the metaclass bases
#     function() argument 1 must be code, not str
@singleton
class TouchRecorder(object):
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
        #Kivy shapes can't be pickled, but as of 1.10, they're only ever either rectangular or None
        if event.shape is not None:
            shape = {"width" : event.shape.width, "height" : event.shape.height}
        else:
            shape = None

        event = {"time": event.time_update,
                 "uid": event.uid,
                 "start_time" : event.time_start,
                 "end_time" : event.time_end,
                 "event_x" : event.x,
                 "event_y" : event.y,
                 "update_time" : event.time_update,
                 "shape" : shape}
        pickle.dump(event, self.outfile)
        #Paranoia
        self.outfile.flush()

    def log_meta_event(self, desc):
        event = {"time": time.time(),
                 "desc": desc}
        pickle.dump(event, self.outfile)
        #Paranoia
        self.outfile.flush()

@singleton
class ROSTouchRecorder(object):
    def __init__(self, prefix=None):
        self.touch_pub = rospy.Publisher('touches', PointStamped, queue_size=10)
        self.meta_pub = rospy.Publisher('meta_events', ROSStrMsg, queue_size=10)

    #Timestamps are in unix time, seconds since the epoch, down to 10ths of a second. 
    def log_touch_event(self, event):
        #Create a point and publish it. This loses a lot of the event data, 
        #but the superclass is logging that
        ps = PointStamped()
        ps.header.frame_id = str(event.uid)
        ps.point.x = event.x
        ps.point.y = event.y
        self.touch_pub.publish(ps)

    #Just publish event messages as strings
    def log_meta_event(self, desc):
        self.meta_pub.publish(ROSStrMsg(desc))

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
        self.rtr = ROSTouchRecorder()
        self.tr = TouchRecorder()

        #Set ourselves up with the inital image
        self.source = self.cfg.get('FilePath', 'path') + self.cfg.get(self.condition, str(self.slideIndex))
        
        #Resize to match the initial image. They're all the same size, so this is legit
        img = Image.open(self.source)
        width, height = img.size
        Window.size = (width, height)

        #Log that we loaded it and refresh the view
        self.tr.log_meta_event("Loaded {0}".format(self.source))
        self.rtr.log_meta_event("Loaded {0}".format(self.source))
        self.canvas.ask_update()

    def nextSlide(self):
        #Increment the slide index and wrap if needed
        self.slideIndex += 1
        if self.slideIndex > self.slideCount:
            self.slideIndex = 1
        #New image for the background
        self.source = self.cfg.get('FilePath', 'path') + self.cfg.get(self.condition, str(self.slideIndex))

        #Log what file was loaded
        self.tr.log_meta_event("Loaded {0}".format(self.source))
        self.rtr.log_meta_event("Loaded {0}".format(self.source))

        #Widget is the same size as the image
        self.canvas.ask_update()

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)
            self.rtr.log_touch_event(touch)

    def on_touch_up(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)
            self.rtr.log_touch_event(touch)

    def on_touch_move(self, touch):
        if self.collide_point(*touch.pos):
            self.tr.log_touch_event(touch)
            self.rtr.log_touch_event(touch)


class ROSMultiImage(MultiImage):
    def __init__(self, **kwargs):
        super(ROSMultiImage, self).__init__(**kwargs)
        self.imgPub = rospy.Publisher("ui_image", ROSImgMsg, queue_size=2)
        #Apparently self exists as soon as init is called, so I can call object methods from init!
        self.pubImage()

    def nextSlide(self):
        super(ROSMultiImage, self).nextSlide()
        self.pubImage()

    def pubImage(self):
        #Superclass sets self.source, load that and put it in a ROS image message
        img = Image.open(self.source)
        
        #From https://github.com/CURG-archive/ros_rsvp/blob/master/image_converter.py
        PIL_MODE_CHANNELS = {'L': 1, 'RGB': 3, 'RGBA': 4, 'YCbCr': 3}
        ENCODINGMAP_PY_TO_ROS = {'L': 'mono8', 'RGB': 'rgb8', 'RGBA': 'rgba8', 'YCbCr': 'yuv422'}

        if img.mode == 'P':
            img = img.convert('RGB')

        rosimage = ROSImgMsg()
        rosimage.encoding = ENCODINGMAP_PY_TO_ROS[img.mode]
        (rosimage.width, rosimage.height) = img.size
        rosimage.step = (PIL_MODE_CHANNELS[img.mode] * rosimage.width)
        rosimage.data = img.tostring()
        #Ship it!
        self.imgPub.publish(rosimage)

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
        self.rtr = ROSTouchRecorder()
        self.tr = TouchRecorder()

    def _keyboard_closed(self):
        self._kbrd.unbind(on_key_down=self._on_keyboard_down)
        self._kbrd = None

    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        # Keycode is composed of an integer + a string
        if keycode[1] == 'n':
            self.tr.log_meta_event("Advanced slide")
            self.rtr.log_meta_event("Advanced slide")
            #TODO THIS FAILS IF THE WIDGET TREE CHANGES
            self.parent.ids["slide_show"].nextSlide()
            #self.parent.ids["finger_draw"].clean_up()
        if keycode[1] == 'c':
            self.tr.log_meta_event("Cleared screen for user")
            self.rtr.log_meta_event("Cleared screen for user")
            #self.parent.ids["finger_draw"].clean_up()
        if keycode[1] == 'q':
            self.tr.log_meta_event("Quit experiment")
            self.rtr.log_meta_event("Quit experiment")
            keyboard.release()
            App.get_running_app().stop()
            #self.parent.ids["finger_draw"].clean_up()
        if keycode[1] == 'i':
            self.tr.log_meta_event("That's interesting")
            self.rtr.log_meta_event("That's interesting")
        # Return True to accept the key. Otherwise, it will be used by
        # the system.
        return True

class UITestApp(App):

    def __init__(self, **kwargs):
        #import pdb; pdb.set_trace()
        super(UITestApp, self).__init__(**kwargs)
        self.condition = kwargs['condition']
        self.subject = kwargs['subject']
        self.images = kwargs["imagepath"] #For some reason this doesn't work like the other two

        print self.condition, self.subject

    def build_config(self, config):
        #If you don't set any defaults, Kivy won't load your config at all 
        config.setdefaults('Condition', {'type': 'files_unknown'})
        config.setdefaults('Subject', {'id': '--undef--'})
        config.setdefaults('FilePath', {'path': '--undef--'})

        print self.condition == 10, self.subject
        #Using the parameters, set the configuration section
        if self.condition == 1:
            config.set('Condition', 'type', 'files_single')
        if self.condition == 10:
            config.set('Condition', 'type', 'files_10')
        if self.condition == 100:
            config.set('Condition', 'type', 'files_100')
        if self.condition == 1000:
            config.set('Condition', 'type', 'files_1000')
        if self.condition == 'X':
            config.set('Condition', 'type', 'files_unknown')

        config.set("Subject", 'id', self.subject)
        config.set('FilePath', 'path', self.images)

    def build(self):
        pass

    def on_pause(self):
        #This is where I'd save the log file
        return True

    def on_stop(self):
        #Also dump the log here
        return True


if __name__ == '__main__':
    # #Kivy apparently ignores things after --
    # import argparse
    # conditions = ['1', '10', '100', '1000','X']
    # parser = argparse.ArgumentParser(description = "Display and log contact points for PhD experiment")
    # parser.add_argument('-i', nargs='?', default=0, type=int, required=True, help='Numerical subject identifier')
    # helptext = "One of " + ", ".join(conditions)
    # parser.add_argument('-c', nargs='?', required=True, help=helptext)
    # args = parser.parse_args()

    # condition = 0
    # subject = 0

    # #Check if the condition argument makes sense. 
    # if vars(args)['c'] not in conditions:
    #     print "Condition must be one of " + ", ".join(conditions)
    #     sys.exit(-1)
    # else:
    #     condition = vars(args)['c']
    
    # subject = vars(args)['i']

    rospy.init_node("multitouch_user_interface")

    condition = rospy.get_param("/ui/cond") 
    subject = rospy.get_param("/ui/id")
    img_path = rospy.get_param("/ui/fpath")

    print "Subject: {0}, Condition {1}".format(subject, condition)

    UITestApp(condition = condition, subject = subject, imagepath = img_path).run()
