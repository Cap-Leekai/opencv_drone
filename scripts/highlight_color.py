#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
# топики
cam_topic = "/mono_cam_forward/camera_mono/image_raw"
ros_img = Image()

def img_cb(data):
    global ros_img
    ros_img = data


# создаём пустую функцию
def nothing(x):
    pass
# делаем захват видео с камеры в переменную cap
# cap = cv.VideoCapture("passing_gazebo_green.mp4")    #stereo elp >> /dev/video2, /dev/video4
# cap.set(cv.CAP_PROP_FPS, 24) # Частота кадров
# cap.set(cv.CAP_PROP_FRAME_WIDTH, 640) # Ширина кадров в видеопотоке.
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, 360) # Высота кадров в видеопотоке.

# создам пустое окно с именем result
cv.namedWindow('result')

# создаём в окне result бегунки для задания порогов цвета
cv.createTrackbar('minb', 'result', 0, 255, nothing)
cv.createTrackbar('ming', 'result', 0, 255, nothing)
cv.createTrackbar('minr', 'result', 0, 255, nothing)

cv.createTrackbar('maxb', 'result', 0, 255, nothing)
cv.createTrackbar('maxg', 'result', 0, 255, nothing)
cv.createTrackbar('maxr', 'result', 0, 255, nothing)

# cv_img = cv.imread('/home/leekay/catkin_ws/src/opencv_drone/images/frame_ang.png')

# cv.imshow('color', color)

def main():
    global ros_img
    rospy.init_node("highlight_color_node")

    bridge = CvBridge()

    rospy.Subscriber(cam_topic, Image, img_cb)

    while not rospy.is_shutdown():
        try:
            cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
            ret = True
            # читаем флаг подключения камеры и картинку с камеры
            # ret, frame = cap.read()

            # ret = True
            # frame = color

            # проверяем есть ли соединение с камерой
            if ret:

                # переводим картинку с камеры из формата BGR в HSV
                # hsv = cv_img

                # hsv = r_channel
                hls = cv.cvtColor(cv_img, cv.COLOR_BGR2HLS)
                # cv.imshow('frame', hsv) # выводим картинку с камеры в формате HSV на экран
                r_channel = hls[ :, :, 1]
                binary_s = np.zeros_like(r_channel)
                binary_s[(r_channel < 25)] = 255

                # Уменьшаем контуры белых объектов - делаем две итерации
                maskEr = cv.erode(binary_s, None, iterations=1)
                # cv.imshow("Erode", maskEr)

                # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
                binary_s = cv.dilate(maskEr, None, iterations=1)
                # cv.imshow('Dilate', maskDi)
                cv.imshow("binary_s", binary_s)

                # получаем значения задаваемые бегунками
                minb = cv.getTrackbarPos('minb', 'result')          #maxb = 118/119
                ming = cv.getTrackbarPos('ming', 'result')          #maxg = 83/105
                minr = cv.getTrackbarPos('minr', 'result')          #maxr = 126/64

                maxb = cv.getTrackbarPos('maxb', 'result')
                maxg = cv.getTrackbarPos('maxg', 'result')
                maxr = cv.getTrackbarPos('maxr', 'result')

                # делаем размытие картинки HSV
                # hsv = cv.blur(hsv, (4, 4))
                # cv.imshow('Blur', hsv)

                # делаем бинаризацию картинки и пихаем её в переменную mask
                mask = cv.inRange(cv_img, (minb, ming, minr), (maxb, maxg, maxr))         #mask = cv.inRange(cv_img, (minb, ming, minr), (maxb, maxg, maxr))
                # cv.imshow('mask', mask)

                # Уменьшаем контуры белых объектов - делаем две итерации
                maskEr = cv.erode(mask, None, iterations=3)
                # cv.imshow("Erode", maskEr)

                # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
                maskDi = cv.dilate(maskEr, None, iterations=3)
                # cv.imshow('Dilate', maskDi)

                # накладываем полученную маску на картинку с камеры переведённую в формат HSV
                # result = cv.bitwise_and(cv_img, cv_img, mask = mask)
                # result = cv.resize(result, (500, 500))

                # binary_hls = hls[:, :, 0]
                #
                # binary = np.zeros_like(cv_img)
                #
                # binary[(binary_hls < 120)] = 255

                cv.imshow('result', mask)

                # print(result)
                if cv.waitKey(1) == 27: # проверяем была ли нажата кнопка esc
                    break
            else:
                print("Camera not found!")
                break
        except:
            print
if __name__ == "__main__":
    main()
    # cap.release()
    cv.destroyAllWindows()