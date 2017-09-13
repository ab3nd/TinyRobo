#!/usr/bin/python

#A Kivy app with a ROS camera feed as the background
import kivy
kivy.require('1.9.1')

#Don't resize the window
#This has to be before any other Kivy imports, or it fails quietly
from kivy.config import Config
Config.set('graphics', 'resizable', False)
#Config.set('graphics', 'fullscreen', 'fake')
Config.set('graphics', 'borderless', False)
from kivy.app import App

from kivy.core.window import Window
from kivy.uix.widget import Widget
from kivy.uix.image import Image as kvImage
from kivy.graphics.texture import Texture

from kivy.core.image import Image as kvCoreImg

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge, CvBridgeError
import io
import cv2
from array import array

#TODO this should probably be customizable
cam_topic = "/usb_cam/image_raw"

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

class ROSImage(kvImage):
    def __init__(self, **kwargs):
        super(ROSImage, self).__init__(**kwargs)
        
        #For converting the image
        self.bridge = CvBridge()

        #Subscribe to images from the overhead camera
        self.cfg = Config.get_configparser('app')
        print self.cfg.get("CamTopic", "path")
        rospy.Subscriber(self.cfg.get("CamTopic", "path"), rosImage, self.update_image)
        
    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            pass

    def on_touch_up(self, touch):
        if self.collide_point(*touch.pos):
            pass

    def on_touch_move(self, touch):
        if self.collide_point(*touch.pos):
            pass

    def update_image(self, msg):
        #Kivy uses this for loading images in memory:
        #data = io.BytesIO(open("image.png", "rb").read())
        #im = CoreImage(data, ext="png")
        #A ros image message is of the form 
        # This message contains an uncompressed image
        # # (0, 0) is at top-left corner of image
        # #

        # Header header        # Header timestamp should be acquisition time of image
        #                      # Header frame_id should be optical frame of camera
        #                      # origin of frame should be optical center of cameara
        #                      # +x should point to the right in the image
        #                      # +y should point down in the image
        #                      # +z should point into to plane of the image
        #                      # If the frame_id here and the frame_id of the CameraInfo
        #                      # message associated with the image conflict
        #                      # the behavior is undefined

        # uint32 height         # image height, that is, number of rows
        # uint32 width          # image width, that is, number of columns

        # # The legal values for encoding are in file src/image_encodings.cpp
        # # If you want to standardize a new string format, join
        # # ros-users@lists.sourceforge.net and send an email proposing a new encoding.

        # string encoding       # Encoding of pixels -- channel meaning, ordering, size
        #                       # taken from the list of strings in include/sensor_msgs/image_encodings.h

        # uint8 is_bigendian    # is this data bigendian?
        # uint32 step           # Full row length in bytes
        # uint8[] data          # actual matrix data, size is (step * rows)

        #cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        #success, img_buffer = cv2.imencode('.png', cv_img)

        #So in theory, I can copy the data into a BytesIO object, and display that...
        #img_data = io.BytesIO(img_buffer.flatten())
        #img_data.seek(0)
        #img = kvCoreImg(img_buffer.to_string(), ext="png")
        print "1"
        tex = Texture.create(size=(64,64), colorfmt='rgb')
        print "2"
        arr = array('B', msg.data[:64*64*3])
        print "3"
        tex.blit_buffer(arr, colorfmt='rgb', bufferfmt='ubyte')
        print "4"
        #Write that image to my canvas
        with self.canvas:
            Rectangle(texture = tex)#, pos=self.pos, size=self.size)

            
        #Attempt to create a buffer and use it results in segfault
        # texture = Texture.create(size=(msg.width, msg.height), colorfmt ='rgb', bufferfmt='ubyte')
        # success, img_buffer = cv2.imencode('.jpg', img)
        # # then blit the buffer
        # texture.blit_buffer(img_buffer.flatten())
        # self.texture = texture
        self.canvas.ask_update()
        print "Ended"
        
class RobotUIMainApp(App):
    def __init__(self, **kwargs):
        super(RobotUIMainApp, self).__init__(**kwargs)
        
    def build_config(self, config):
        #If you don't set any defaults, Kivy won't load your config at all 
        config.setdefaults('CamTopic', {'path': cam_topic})
        
    def build(self):
        pass

    def on_pause(self):
        #Not sure this should do anything
        return True

    def on_stop(self):
        #Unsubscribe from ROS messages
        return True


if __name__ == '__main__':
    rospy.init_node('RobotUI')
    RobotUIMainApp().run()