from cv2 import VideoCapture

class Camera(object):
    def __init__(self, camera=0):
        self.camera = VideoCapture(camera)

    def __enter__(self):
        return self

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
