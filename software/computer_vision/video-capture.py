from cv2 import VideoCapture, cvtColor, imshow, resizeWindow, WINDOW_NORMAL, destroyAllWindows

from lib.camera import Camera
from lib.bluedetection import *
from lib.keycodes import *

def frame_filter(key, frame):
    filters = {
        key_w: find_lines,
        key_e: cmask,
        key_r: cmask_and_erode,
        key_t: find_lines_in_eroded_cmask,
        key_y: cmask_and_find_lines
    }   
    if key in filters:
        return filters[key](frame)
    else:
        return frame

def main():
    name, width, height = 'Camera Filter Test', 1280, 720

    namedWindow(name, WINDOW_NORMAL)
    resizeWindow(name, width, height)

    with Camera() as camera:
        set_key = None
        for frame in camera:
            pressed_key = key()

            if pressed_key == key_q:
                break
            if not set_key == pressed_key and not pressed_key == key_none:
                set_key = pressed_key

            imshow(name, frame_filter(set_key, frame))

    destroyAllWindows()

if __name__ == '__main__':
    main()
