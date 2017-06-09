from cv2 import VideoCapture

class Camera(object):
    def __init__(self):
        self.camera = VideoCapture(0)

    def __enter__(self):
        pass

    def __exit__(self, eType, eValue, eTrace):
        self.close()

    def frame(self):
        return self.camera.read()

    def __iter__(self):
        while True:
            ret, frame = self.frame()
            if ret:
                yield frame
            else:
                break  

    def close(self):
        self.camera.release()
