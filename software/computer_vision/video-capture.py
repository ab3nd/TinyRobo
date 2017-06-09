from cv2 import VideoCapture, cvtColor, imshow, resizeWindow, WINDOW_NORMAL, destroyAllWindows

from lib.camera import Camera
from lib.bluedetection import *
from lib.keycodes import keypress, key

def frame_filter(keypress, frame):
    filters = {
        key.w: find_lines,
        key.e: cmask,
        key.r: cmask_and_erode,
        key.y: find_lines_in_eroded_cmask,
        key.u: cmask_and_find_lines
    }   
    if keypress in filters:
        return filters[keypress](frame)
    else:
        return frame

def main():
    name, width, height = 'Camera Filter Test', 1280, 720

    namedWindow(name, WINDOW_NORMAL)
    resizeWindow(name, width, height)

    with Camera() as camera:
        set_key = None
        for frame in camera:
            pressed_key = keypress()

            if pressed_key == key.q:
                break
            if not set_key == pressed_key and not pressed_key == key.none:
                set_key = pressed_key

            imshow(name, frame_filter(set_key, frame))

    destroyAllWindows()

if __name__ == '__main__':
    main()
