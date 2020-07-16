#!/usr/bin/env python
#coding=utf8

import cv2 as cv

cap = cv.VideoCapture("/dev/video6") # "/dev/video0"

while True:

    ret, frame = cap.read()
    print ret

    if ret:
        cv.imshow("hnya", frame)

        #print(result)
        if cv.waitKey(1) == 27: # проверяем была ли нажата кнопка esc
            break
    else:
        print("Camera not found!")
        break

cap.release()
cv.destroyAllWindows()
