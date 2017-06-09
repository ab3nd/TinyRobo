from cv2 import VideoCapture, cvtColor, imshow, COLOR_BGR2GRAY, waitKey, destroyAllWindows

from lib.camera import Camera
from lib.bluedetection import *

key_q, key_w, key_e, key_r = ord('q'), ord('w'), ord('e'), ord('r')

def key():
    return waitKey(1) & 0xFF

def frame_filter(key, frame):
    filters = {
        key_w: find_lines,
        key_e: cmask,
        key_r: cmask_and_find_lines
    }   
    if key in filters:
        return filters[key](frame)
    else:
        return frame

with Camera() as camera:
    set_key = None
    for frame in camera:
        pressed_key = key()

        if pressed_key == key_q:
            break
        if not set_key == pressed_key and not pressed_key == 255:
            set_key = pressed_key

        imshow('frame', frame_filter(set_key, frame))

destroyAllWindows()
