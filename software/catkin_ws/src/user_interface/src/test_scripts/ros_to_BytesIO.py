#!/usr/bin/python

#For test, remove later
from PIL import Image as PILImage

import numpy as np
from io import BytesIO  

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image

class Converter(object):

    def __init__(self, **kwargs):
        #intialize ROS and subscribe to an image topic
        topic = "/overhead_cam/image_rect_color"
        rospy.init_node('img_mauler')

        self.sub = rospy.Subscriber(topic, Image, self.update_image)

    def update_image(self, imgMsg):
        file = BytesIO()
        image = PILImage.new('RGBA', size=(50, 50), color=(155, 255, 0))
        image.save(file, 'png')
        file.name = 'test.png'

        print "Did the thing!"


if __name__ == '__main__':
    c = Converter()
    rospy.spin()
