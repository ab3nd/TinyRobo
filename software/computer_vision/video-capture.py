from cv2 import VideoCapture, cvtColor, imshow, COLOR_BGR2GRAY, waitKey, destroyAllWindows

from lib.camera import Camera
from lib.bluedetection import *
camera = Camera()

for frame in camera:
    #gray = find_lines(frame)
    imshow('frame', frame)
    if waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
destroyAllWindows()
