#!/usr/bin/python

import kivy
kivy.require('1.9.1') # replace with your current kivy version !
from kivy.config import Config
from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.image import Image as UIXImage
from kivy.core.image import Image as CoreImage
from kivy.graphics import Rectangle
from kivy.clock import Clock
from kivy.base import EventLoop
from kivy.uix.floatlayout import FloatLayout
from kivy.core.window import Window

#For image conversion to Kivy textures
from PIL import Image as PILImage
from io import BytesIO  

#For ROS interfacing
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

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
        self.touch_pub = rospy.Publisher('touches', PointStamped, queue_size=10)

    #Timestamps are in unix time, seconds since the epoch, down to 10ths of a second. 
    def log_touch_event(self, event):
        #Create a point and publish it. This loses a lot of the event data, 
        ps = PointStamped()
        ps.header.frame_id = str(event.uid)
        ps.point.x = event.x
        ps.point.y = event.y
        self.touch_pub.publish(ps)


class ROSTouchImage(UIXImage):
    def __init__(self, **kwargs):
        super(UIXImage, self).__init__(**kwargs)
        
        #Record touch events
        self.rtr = ROSTouchRecorder()
        
        #Resize to match the initial image. They're all the same size, so this is legit
        # img = Image.open(self.source)
        # width, height = img.size
        # Window.size = (width, height)

        self.canvas.ask_update()

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            self.rtr.log_touch_event(touch)

    def on_touch_up(self, touch):
        if self.collide_point(*touch.pos):
            self.rtr.log_touch_event(touch)

    def on_touch_move(self, touch):
        if self.collide_point(*touch.pos):
            self.rtr.log_touch_event(touch)

class StupidApp(App):

    def __init__(self, **kwargs):
        super(StupidApp, self).__init__(**kwargs)

        #intialize ROS and subscribe to an image topic
        topic = "/overhead_cam/image_rect_color"
        rospy.init_node('kivy_img_mauler')
        #We can get away with not calling rospy.Spin() because Kivy keeps it running
        self.sub = rospy.Subscriber(topic, Image, self.update_image)
        
        self.rosImage = None
        self.width = 1024
        self.height = 768
        
        EventLoop.ensure_window()
        Clock.schedule_interval(self.display_image, 1.0 / 30.0)
        
    def build(self):
        self.layout = FloatLayout()
        try:
            self.uiImage = ROSTouchImage()
        except Exception as e:
            print e
        self.layout.add_widget(self.uiImage)
        return self.layout

    def display_image(self, dt):
        try:
            if self.rosImage is None:
                return
            imageData = BytesIO()
            self.rosImage.save(imageData, "PNG")
            imageData.seek(0)
            im = CoreImage(imageData, ext='png')

            self.uiImage.canvas.clear()
            with self.uiImage.canvas:
                #NOTE: On implementions that don't support NPOT (non-power-of-two) textures
                #this will cause a problem, probably a segmentation fault
                Rectangle(texture = im.texture, size=(im.texture.width, im.texture.height))

            #Set our window size to the size of the texture
            Window.size = (im.texture.width, im.texture.height)

        except Exception as e:
            print e

        return True

    def update_image(self, imgMsg):
        self.rosImage = ImageConverter.from_ros(imgMsg)
        return True

    def on_pause(self):
        #Not sure this should do anything
        return True

    def on_stop(self):
        #Unsubscribe from ROS messages
        return True

if __name__ == '__main__':
    StupidApp().run()

#Might come in handy later
# from threading import Thread
# spin_thread = Thread(target=lambda: rospy.spin())
# spin_thread.start()