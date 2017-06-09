from cv2 import VideoCapture, cvtColor, imshow, COLOR_BGR2GRAY, waitKey, destroyAllWindows

from lib.camera import Camera
from lib.bluedetection import *

def frame_filter(key, frame):
    filters = {
        119: find_lines, # 'w'
        101: mask        # 'e'
    }   
    if key in filters:
        return filters[key](frame)
    else:
        return frame

with Camera() as camera:
    set_key = None
    for frame in camera:
        pressed_key = waitKey(1) & 0xFF

        if pressed_key == 113: # 'q'
            break
        if not set_key == pressed_key and not pressed_key == 255:
            set_key = pressed_key

        imshow('frame', frame_filter(set_key, frame))

destroyAllWindows()
