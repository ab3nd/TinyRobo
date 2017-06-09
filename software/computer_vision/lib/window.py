from cv2 import namedWindow, WINDOW_NORMAL, resizeWindow, destroyWindow

class Window(object):
    def __init__(self, name, width=1280, height=720):
        self._name = name
        self._width = width
        self._height = height
        namedWindow(name, WINDOW_NORMAL)
        resizeWindow(name, width, height)

    def __enter__(self):
        return self

    def __exit__(self, eType, eValue, eTrace):
        destroyWindow(self._name)        
