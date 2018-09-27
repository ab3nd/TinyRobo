#!/usr/bin/python

import kivy
kivy.require('1.9.1') # replace with your current kivy version !
from kivy.config import Config
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout
from kivy.uix.image import Image as UIXImage
from kivy.core.image import Image as CoreImage
from kivy.graphics import Rectangle, Ellipse
from kivy.clock import Clock
from kivy.base import EventLoop
from kivy.uix.floatlayout import FloatLayout
from kivy.core.window import Window

#For image conversion to Kivy textures
from PIL import Image as PILImage
from PIL import ImageDraw
from io import BytesIO  
import threading

#For ROS interfacing
import rospy
from sensor_msgs.msg import Image
from apriltags_ros.msg import *
from user_interface.msg import Kivy_Event, Gesture


class ImageConverter(object):
    """
    Based on https://github.com/CURG-archive/ros_rsvp/blob/master/image_converter.py
    Convert images/compressedimages to and from ROS
    """

    _ENCODINGMAP_PY_TO_ROS = {'L': 'mono8', 'RGB': 'rgb8',
                              'RGBA': 'rgba8', 'YCbCr': 'yuv422'}
    _ENCODINGMAP_ROS_TO_PY = {'mono8': 'L', 'rgb8': 'RGB',
                              'rgba8': 'RGBA', 'yuv422': 'YCbCr'}
    _PIL_MODE_CHANNELS = {'L': 1, 'RGB': 3, 'RGBA': 4, 'YCbCr': 3}

    @staticmethod
    def to_ros(img):
        """
        Convert a PIL/pygame image to a ROS compatible message (sensor_msgs.Image).
        """

        # Everything ok, convert PIL.Image to ROS and return it
        if img.mode == 'P':
            img = img.convert('RGB')

        rosimage = sensor_msgs.msg.Image()
        rosimage.encoding = ImageConverter._ENCODINGMAP_PY_TO_ROS[img.mode]
        (rosimage.width, rosimage.height) = img.size
        rosimage.step = (ImageConverter._PIL_MODE_CHANNELS[img.mode]
                         * rosimage.width)
        rosimage.data = img.tostring()
        return rosimage

    @staticmethod
    def from_ros(rosMsg):
        """
        Converts a ROS sensor_msgs.Image to a PIL image
        :param rosMsg: The message to convert
        :return: an alpha-converted pygame Surface
        """
        return PILImage.frombytes(ImageConverter._ENCODINGMAP_ROS_TO_PY[rosMsg.encoding], (rosMsg.width, rosMsg.height), rosMsg.data)

class ROSTouchRecorder(object):
    def __init__(self, prefix=None):
        self.touch_pub = rospy.Publisher('touches', Kivy_Event, queue_size=10)
        
    def log_touch_event(self, event):
        #Create an event message and publish that
        em = Kivy_Event()
        em.uid = event.uid
        em.point.x = event.x
        em.point.y = 1050 - event.y #Flip kivy coords into pixel coords
        em.point.z = event.z
        em.isTripletap = event.is_triple_tap
        em.isDoubletap = event.is_double_tap
        em.start = rospy.Time.from_sec(event.time_start)
        if event.time_end < 0:
            #Kivy uses -1 to mean not ended yet, so set it to the most recent time
            em.end = rospy.Time.from_sec(event.time_update)
            em.ended = False
        else:
            em.end = rospy.Time.from_sec(event.time_end)
            em.ended = True
        em.update = rospy.Time.from_sec(event.time_update)
        self.touch_pub.publish(em)


class ROSTouchImage(UIXImage):
    def __init__(self, **kwargs):
        super(UIXImage, self).__init__(**kwargs)
        
        #Record touch events
        self.rtr = ROSTouchRecorder()
        
        #Keep track of whether this is being touched
        self.contacts = {}
        self.isTouched = False

        self.canvas.ask_update()

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            self.rtr.log_touch_event(touch)

            #Keep track of this touch
            self.contacts[touch.uid] = touch
            self.isTouched = True

    def on_touch_up(self, touch):
        if self.collide_point(*touch.pos):
            self.rtr.log_touch_event(touch)

            #Delete the touch if it's still around
            if touch.uid in self.contacts.keys():
                del self.contacts[touch.uid]
            #If no touches remain, nothing is touching anymore
            if len(self.contacts.keys()) == 0:
                self.isTouched = False

    def on_touch_move(self, touch):
        if self.collide_point(*touch.pos):
            self.rtr.log_touch_event(touch)

class OverheadUIApp(App):

    def __init__(self, **kwargs):
        super(OverheadUIApp, self).__init__(**kwargs)

        #intialize ROS and subscribe to an image topic
        rospy.init_node("ui") #Gets overwritten by launchfile
        topic = rospy.get_param("{}/overhead_cam".format(rospy.get_name()), "/overhead_cam/image_rect_color")
        
        #We can get away with not calling rospy.Spin() because Kivy keeps it running
        self.sub = rospy.Subscriber(topic, Image, self.update_image)
        #Gesture recognizers also publish on /gestures
        self.button_pub = rospy.Publisher("gestures", Gesture, queue_size=10)

        self.rosImage = None
        self.kivyImage = None

        EventLoop.ensure_window()
        Clock.schedule_interval(self.display_image, 1.0 / 3.0)
        Clock.schedule_interval(self.convert_image, 1.0 / 3.0)
        
    def build(self):
        self.layout = FloatLayout()
        try:
            self.uiImage = ROSTouchImage()
        except Exception as e:
            print e
        self.layout.add_widget(self.uiImage)

        #Add buttons for more complex actions
        #Patrol
        self.patrol = Button(text="Patrol", pos_hint={'right': 0.1, "top": 0.98}, size_hint=(0.07, 0.05))
        self.patrol.bind(on_press=self.handle_button)
        self.layout.add_widget(self.patrol)
        #Formation
        self.formation = Button(text="Formation", pos_hint={'right': 0.2, "top": 0.98}, size_hint=(0.07, 0.05))
        self.formation.bind(on_press=self.handle_button)
        self.layout.add_widget(self.formation)
        #Move object
        self.move_obj = Button(text="Move Object", pos_hint={'right': 0.3, "top": 0.98}, size_hint=(0.07, 0.05))
        self.move_obj.bind(on_press=self.handle_button)
        self.layout.add_widget(self.move_obj)
        #Remove robot
        self.remove_bot = Button(text="Remove Robot", pos_hint={'right': 0.4, "top": 0.98}, size_hint=(0.07, 0.05))
        self.remove_bot.bind(on_press=self.handle_button)
        self.layout.add_widget(self.remove_bot)
        #Select by color
        self.select_color = Button(text="Select Group", pos_hint={'right': 0.5, "top": 0.98}, size_hint=(0.07, 0.05))
        self.select_color.bind(on_press=self.handle_button)
        self.layout.add_widget(self.select_color)
        return self.layout

    def handle_button(self, instance):
        evt = Gesture()
        evt.stamp = rospy.Time.now()
        evt.eventName = instance.text.lower().replace(" ", "_")
        self.button_pub.publish(evt)
        return True

    def update_tags(self, tag_msg):
        self.tags = tag_msg
        return True

    def display_image(self, dt):
        if self.kivyImage is not None:
            self.uiImage.canvas.clear()
            with self.uiImage.canvas:
                #NOTE: On implementions that don't support NPOT (non-power-of-two) textures
                #this will cause a problem, probably a segmentation fault
                Rectangle(texture = self.kivyImage.texture, size=(self.kivyImage.texture.width, self.kivyImage.texture.height))
                
            #Set our window size to the size of the texture
            Window.size = (self.kivyImage.texture.width, self.kivyImage.texture.height)
        return True

    def update_image(self, imgMsg):
        self.rosImage = imgMsg

    def convert_image(self, dt):
        if self.rosImage is not None and not self.uiImage.isTouched:
            tempImg = ImageConverter.from_ros(self.rosImage)

            #Resize to fit screen
            #No longer needed because image is resized by ROS
            #tempImg = tempImg.crop((0,120,1024,768)).resize((1680, 1050))
            
            #Convert pil image to a kivy textureable image
            imageData = BytesIO()
            tempImg.save(imageData, "PNG")
            imageData.seek(0)
            self.kivyImage = CoreImage(imageData, ext='png')

    def on_pause(self):
        #Not sure this should do anything
        return True

    def on_stop(self):
        return True

if __name__ == '__main__':
    OverheadUIApp().run()

