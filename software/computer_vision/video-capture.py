from cv2 import VideoCapture, cvtColor, imshow, COLOR_BGR2GRAY, waitKey, destroyAllWindows

camera = VideoCapture()

while(True):
        ret, frame = camera.read()
  
        if frame:
            gray = cvtColor(frame, COLOR_BGR2GRAY)
            imshow('frame', gray)
            if waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print 'Camera not working'
            break

camera.release()
destroyAllWindows()
