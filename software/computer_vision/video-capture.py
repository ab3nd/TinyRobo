from cv2 import VideoCapture, cvtColor, imshow, COLOR_BGR2GRAY, waitKey, destroyAllWindows

from lib.bluedetection import *
camera = VideoCapture(0)

while(True):
        ret, frame = camera.read() 
 
        if ret:
            gray = find_lines(frame)
            imshow('frame', gray)
            if waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print 'Camera not working'
            break

camera.release()
destroyAllWindows()
