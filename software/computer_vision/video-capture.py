from lib.camera import Camera
from lib.bluedetection import *
from lib.window import Window
from lib.keycodes import keypress, key

def frame_filter(keypress, frame):
    filters = {
        key.w: find_lines,
        key.e: cmask,
        key.r: cmask_erode,
        key.t: cmask_erode_morph,
        key.y: cmask_erode_morph_find_lines,
        key.u: cmask_find_lines
    }   
    if keypress in filters:
        return filters[keypress](frame)
    else:
        return frame

def main():
    with Window(name='Camera Test', width=1280, height=720) as window:
        with Camera() as camera:
            set_key = None
            for frame in camera:
                pressed_key = keypress()

                if pressed_key == key.q:
                    break
                if not set_key == pressed_key and not pressed_key == key.none:
                    set_key = pressed_key

                window.display(frame_filter(set_key, frame))

if __name__ == '__main__':
    main()
