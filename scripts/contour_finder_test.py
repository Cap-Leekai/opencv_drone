#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import os
import numpy as np

width_of_line = 0.2

def recalculation_cords(AllBinary):
    LIST = []
    for i in range(AllBinary.shape[1]):
        g = 0
        for j in range(AllBinary.shape[0]):
            # print "Столбец -> ", i, "Строка -> ", j
            # print AllBinary[j][i]
            if AllBinary[j][i] > 0:
                g += 1

        # print int(g // 0.2)
        LIST.append(int(g // width_of_line))
    return LIST


def main():
    while True:
        cv_img = cv.imread("/home/leekay/catkin_ws/src/opencv_drone/images/line_img.png")
        # извлекаем КРАСНЫЙ канал из кадра видеопотока
        r_channel = cv_img[:, :, 2]
        # создаем массив размером как r_channel
        binary_r = np.zeros_like(r_channel)
        # заносим соответствующие пиксели из r_channel, которые меньше 2 в массив binary_r со знаением 1
        binary_r[(r_channel < 2)] = 1

        hls_img = cv.cvtColor(cv_img, cv.COLOR_BGR2HLS)

        s_channel = hls_img[:, :, 2]
        binary_s = np.zeros_like(s_channel)
        binary_s[(s_channel < 2)] = 1

        # объединяем два бинаризованного изображения в одно
        AllBinary = np.zeros_like(binary_r)
        AllBinary[((binary_r == 1) | (binary_s == 1))] = 255

        # Уменьшаем контуры белых объектов - делаем 5 итераций
        AllBinary = cv.erode(AllBinary, None, iterations=5)

        # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем 5 итераций
        AllBinary = cv.dilate(AllBinary, None, iterations=5)




        cv.imshow("test", AllBinary)

        # находим контуры
        contours, hierarchy = cv.findContours(AllBinary, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        # сортируем контуры
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        print len(contours)
        # вычленяем массив контуров из переменной contours и переинициализируем переменную contours
        cv.drawContours(cv_img, contours, -1, (0, 180, 255), 1)

        cv.imshow("test", cv_img)


        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break

if __name__ == "__main__":
    print "__BEGIN__"
    main()