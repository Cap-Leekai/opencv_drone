#!/usr/bin/env python
#coding=utf8

import rospy
import cv2 as cv


# задаем пороги цвета
minb = 6
ming = 170
minr = 131

maxb = 84
maxg = 255
maxr = 255

view_window_flag = True

    # делаем захват видео с камеры в переменную cap
cap = cv.VideoCapture(0)  # stereo elp >> /dev/video2, /dev/video4

def main():

    while(True):

        # читаем флаг подключения камеры и картинку с камеры
        ret, frame = cap.read()
        # print(frame)
        if ret:

            # переводим картинку с камеры из формата BGR в HSV
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            #cv.imshow('frame', hsv)  # выводим картинку с камеры в формате HSV на экран

            # делаем размытие картинки HSV
            hsv = cv.blur(hsv, (4, 4))

            if view_window_flag:
                cv.imshow('Blur', hsv)

            # делаем бинаризацию картинки и пихаем её в переменную mask
            mask = cv.inRange(hsv, (minb, ming, minr), (maxb, maxg, maxr))
            #cv.imshow('mask', mask)

            # Уменьшаем контуры белых объектов - делаем две итерации
            mask = cv.erode(mask, None, iterations=2)
            #cv.imshow("Erode", mask)

            # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
            mask = cv.dilate(mask, None, iterations=2)

            if view_window_flag:
                cv.imshow('Dilate', mask)

            # накладываем полученную маску на картинку с камеры переведённую в формат HSV
            result = cv.bitwise_and(frame, frame, mask = mask)

            if view_window_flag:
                cv.imshow('result', result)

            # ищем контуры в результирующем кадре
            contours = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

            # вычленяем массив контуров из переменной contours и переинициализируем переменную contours
            contours = contours[1]

            if contours:
                # сортируем элементы массива контуров по площади по убыванию
                contours = sorted(contours, key = cv.contourArea, reverse = True)
                # выводим все контуры на изображении
                cv.drawContours(frame, contours, -1, (0, 180, 255), 1)  #cv.drawContours(кадр, массив с контурами, индекс контура, цветовой диапазон контура, толщина контура)

                # получаем координаты прямоугольника описанного относительно контура
                (x, y, w, h) = cv.boundingRect(contours[0])
                # рисуем прямоугольник описанный относительно контура
                cv.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

                print("x: %s, y: %s" % (x + (w/2), y + (h/2)))
                # print("frame_center_cords:","x = ", len(frame[0])/2, "y = ", len(frame)/2)

                # рисуем окружность в центре детектируемого прямоугольника
                cv.circle(frame, (x + (w / 2), y + (h / 2)), 5, (0, 255, 0), thickness=2)
                cv.circle(frame,(len(frame[0])/2, len(frame)/2),5, (0, 255, 0), thickness = 2)

                if view_window_flag:
                    cv.imshow('Contours', frame)

                print("Найдено контуров %s" %len(contours))

            else:
                cv.destroyWindow('Contours')

            # print(result)
            if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
                break

        else:
            print("Camera not found!")
            break

if __name__ == "__main__":

    main()
    cap.release()
    cv.destroyAllWindows()