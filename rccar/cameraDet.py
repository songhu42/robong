import cv2
from pop import Util
#Util.enable_imshow()

import time
from pop import Pilot as rc

car = rc.AutoCar()

if not camera.isOpened():
    camera.release()
    cv2.destroyAllWindows()

cam = Util.gstrmer(width=640, height=480)
camera = cv2.VideoCapture(cam, cv2.CAP_GSTREAMER)
curPan = 90

print('Start Program')

if not camera.isOpened():
    print("Not found camera")

while True:
    ret, frame = camera.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces= face_cascade.detectMultiScale(gray, scaleFactor=1.3 ,minNeighbors=1,minSize=(100,100))

        for (x,y,w,h) in faces:
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        
        cv2.imshow("soda", frame) 
    else :
        print('ret is False')
        
    k = cv2.waitKey(1) & 0xff
    
    if k == 27 : # Esc key to quit
        print('key pressed ,')
        break
    elif k == ord('q') :
        print('key pressed q')
        break
    elif k == ord(',') :
        print('key pressed <')
        curPan -= 10
        car.camPan(curPan)
    elif k == ord('.') :
        print('key pressed >')
        curPan += 10
        car.camPan(curPan)
        
camera.release()
cv2.destroyAllWindows()

print('Exit Program')
