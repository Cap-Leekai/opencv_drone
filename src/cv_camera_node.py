#!/usr/bin/env python
#coding=utf8

import cv2 as cv

    #делаем захват картинки с камеры и пихаем её в переменную cap
cap = cv.VideoCapture(0)

while(True):

    #читаем флаг подключения камеры и картинку с камеры
    ret, frame = cap.read()
    # print(frame)

    cv.imshow('Camera_image', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break



cap.release()
cv.destroyAllWindows()