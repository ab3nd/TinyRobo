from cv2 import namedWindow, WINDOW_NORMAL, resizeWindow, imshow, destroyWindow

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
        self.close()

    def display(self, image):
        imshow(self._name, image)

    def close(self):
        destroyWindow(self._name)
