from cv2 import VideoCapture, cvtColor, imshow, COLOR_BGR2GRAY, waitKey, destroyAllWindows

from lib.camera import Camera
from lib.bluedetection import *
camera = Camera()

while(True):
        frame = camera.frame() 
 
        if frame.any():
            gray = find_lines(frame)
            imshow('frame', gray)
            if waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print 'Camera not working'
            break

camera.release()
destroyAllWindows()
