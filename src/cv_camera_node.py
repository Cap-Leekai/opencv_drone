#!/usr/bin/env python
#coding=utf8

import cv2 as cv

# задаем пороги цвета
minb =
ming =
minr =

maxb =
maxg =
maxr =

    # делаем захват видео с камеры в переменную cap
cap = cv.VideoCapture("/dev/video2")  # stereo elp >> /dev/video2, /dev/video4

while(True):

    #читаем флаг подключения камеры и картинку с камеры
    ret, frame = cap.read()
    # print(frame)
    if ret:

        # переводим картинку с камеры из формата BGR в HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        #cv.imshow('frame', hsv)  # выводим картинку с камеры в формате HSV на экран

        # делаем размытие картинки HSV
        hsv = cv.blur(hsv, (4, 4))
        cv.imshow('Blur', hsv)

        # делаем бинаризацию картинки и пихаем её в переменную mask
        mask = cv.inRange(hsv, (minb, ming, minr), (maxb, maxg, maxr))
        #cv.imshow('mask', mask)

        maskEr = cv.erode(mask, None, iterations=2)
        #cv.imshow("Erode", maskEr)

        maskDi = cv.dilate(maskEr, None, iterations=2)
        cv.imshow('Dilate', maskDi)

        # накладываем полученную маску на картинку с камеры переведённую в формат HSV
        result = cv.bitwise_and(frame, frame, mask=mask)
        cv.imshow('result', result)

        # print(result)
        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break

        else:
            print("Camera not found!")
            break


cap.release()
cv.destroyAllWindows()