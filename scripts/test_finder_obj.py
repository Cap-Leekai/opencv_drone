#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import numpy as np
import math


camera_port = "/dev/video2"


def main():
    cap = cv.VideoCapture(camera_port)
    cap.set(cv.CAP_PROP_FPS, 24)  # Частота кадров
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)  # Ширина кадров в видеопотоке.
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 360)  # Высота кадров в видеопотоке.

    while True:
        ret, frame = cap.read()
        if ret:
            # основной блок программы
            cv.imshow('test', frame)

        else:
            print("camera not found")

        if cv.waitKey(1) == 27: # проверяем была ли нажата кнопка esc
            break

if __name__ == "__main__":
    main()