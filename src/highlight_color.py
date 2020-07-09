#!/usr/bin/env python
#coding=utf8

import cv2 as cv

    #создаём пустую функцию
def nothing(x):
    pass

    #делаем захват видео с камеры в переменную cap
cap = cv.VideoCapture("/dev/video0")    #stereo elp >> /dev/video2, /dev/video4

    #создам пустое окно с именем result
cv.namedWindow('result')

    #создаём в окне result бегунки для задания порогов цвета
cv.createTrackbar('minb', 'result', 0, 255, nothing)
cv.createTrackbar('ming', 'result', 0, 255, nothing)
cv.createTrackbar('minr', 'result', 0, 255, nothing)

cv.createTrackbar('maxb', 'result', 0, 255, nothing)
cv.createTrackbar('maxg', 'result', 0, 255, nothing)
cv.createTrackbar('maxr', 'result', 0, 255, nothing)

color = cv.imread('point_land.jpg')
cv.imshow('color', color)

while(True):

    #читаем флаг подключения камеры и картинку с камеры
    # ret, frame = cap.read()

    ret = True
    frame = color


    #проверяем есть ли соединение с камерой
    if ret:

        #переводим картинку с камеры из формата BGR в HSV
        hsv = frame
        # hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        cv.imshow('frame', hsv) #выводим картинку с камеры в формате HSV на экран

        #получаем значения задаваемые бегунками
        minb = cv.getTrackbarPos('minb', 'result')
        ming = cv.getTrackbarPos('ming', 'result')
        minr = cv.getTrackbarPos('minr', 'result')

        maxb = cv.getTrackbarPos('maxb', 'result')
        maxg = cv.getTrackbarPos('maxg', 'result')
        maxr = cv.getTrackbarPos('maxr', 'result')

        #делаем размытие картинки HSV
        hsv = cv.blur(hsv, (4,4))
       # cv.imshow('Blur', hsv)

        #делаем бинаризацию картинки и пихаем её в переменную mask
        mask = cv.inRange(hsv, (minb, ming, minr), (maxb, maxg, maxr))
       # cv.imshow('mask', mask)

        # Уменьшаем контуры белых объектов - делаем две итерации
        maskEr = cv.erode(mask, None, iterations=2)
        # cv.imshow("Erode", maskEr)

        # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
        maskDi = cv.dilate(maskEr, None, iterations=2)
        cv.imshow('Dilate', maskDi)

        # накладываем полученную маску на картинку с камеры переведённую в формат HSV
        result = cv.bitwise_and(frame, frame, mask = mask)
        cv.imshow('result', result)

        #print(result)
        if cv.waitKey(1) == 27: # проверяем была ли нажата кнопка esc
            break
    else:
        print("Camera not found!")
        break

cap.release()
cv.destroyAllWindows()