#!/usr/bin/python

# Subscribe to points and UI messages
# When a new point arrives, draw it on the UI and publish the resulting image

import rospy

from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import PointStamped

#for building the frame
from PIL import Image as PILImage
from PIL import ImageDraw

currentImage = None

#From https://github.com/CURG-archive/ros_rsvp/blob/master/image_converter.py
PIL_MODE_CHANNELS = {'L': 1, 'RGB': 3, 'RGBA': 4, 'YCbCr': 3}
ENCODINGMAP_PY_TO_ROS = {'L': 'mono8', 'RGB': 'rgb8', 'RGBA': 'rgba8', 'YCbCr': 'yuv422'}
ENCODINGMAP_ROS_TO_PY = {'mono8': 'L', 'rgb8': 'RGB','rgba8': 'RGBA', 'yuv422': 'YCbCr'}

# Start the node
rospy.init_node('ui_interactions', anonymous=True)

#Output, publishes the modified images
imgPub = rospy.Publisher("/ui_interactions", ImageMsg, queue_size=10)
    
def drawPoint(pointMsg):
    global currentImage

    #import pdb; pdb.set_trace()

    if currentImage is None:
        return
    #Magic numbers are half of the difference in screen sizes between the UI image
    #and the 3M multitiouch display, so the points end up in the right locations
    x = pointMsg.point.x - 340
    y = pointMsg.point.y - 150

    # Kivy points are upside down from PIL points
    y = max(750 - y, 0)

    #Draw a dot on the screen at the touch point
    draw = ImageDraw.Draw(currentImage)
    dotSize = 2
    draw.ellipse([(x-dotSize, y-dotSize), (x+dotSize, y+dotSize)], fill='blue')
    del draw

    #Convert from a PIL image to a ROS image message
    #First make sure it's RGB
    if currentImage.mode == 'P':
        currentImage = currentImage.convert('RGB')

    #Fill in the ROS image message
    rosimage = ImageMsg()
    rosimage.encoding = ENCODINGMAP_PY_TO_ROS[currentImage.mode]
    (rosimage.width, rosimage.height) = currentImage.size
    rosimage.step = (PIL_MODE_CHANNELS[currentImage.mode] * rosimage.width)
    rosimage.data = currentImage.tobytes()
    #Ship it!
    imgPub.publish(rosimage)

def updateImage(imgMsg):
    global currentImage
    #Convert to a PIL image, mapping is from https://github.com/CURG-archive/ros_rsvp/blob/master/image_converter.py
    encoding = ENCODINGMAP_ROS_TO_PY[imgMsg.encoding]
    #This overwrites the previous drawn points (I hope...)
    #currentImage = PILImage.fromstring(encoding, (imgMsg.width, imgMsg.height), imgMsg.data)
    currentImage = PILImage.frombytes(encoding, (imgMsg.width, imgMsg.height), imgMsg.data)
    #Republish the un-edited image
    imgPub.publish(imgMsg)

#Subscribe to changes in the UI screen and touch points
pointSub = rospy.Subscriber("/touches", PointStamped, drawPoint)
imgSub = rospy.Subscriber("/ui_image", ImageMsg, updateImage)

rospy.spin()