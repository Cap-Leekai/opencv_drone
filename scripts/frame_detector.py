#!/usr/bin/env python
#coding=utf8


import cv2 as cv
import numpy as np
import rospy
import rospkg
import os
import copy
import time


from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage


depth_image_topic = "/r200/depth/image_raw"
image_topic = "/r200/image_raw"


depth_frame = None
image_binary = None

# callback считывания картинки с realsence в rgb
def rgb_image_cb(data):
    global rgb_image
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")

# callback считывания карты глубины с realsence
def depth_image_cb(data):
    global image_binary, depth_frame
    try:
        bridge = CvBridge()

        # переводим фрейм из росовского сообщения в картинку opencv
        depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
        depth_frame = np.array(depth_image, dtype=np.float32)   # каждый элемент фрейма хранит значение типа float являющееся расстоянием в метрах до точки

        image_binary = np.zeros_like(depth_frame)
        # делаем маску из допустимых пикселей на основе условия
        image_binary[(depth_frame < 5.)] = 255
        image_binary = np.uint8(image_binary)

        # cv.imshow("depth", image_binary)
    except CvBridgeError as e:
        print "Error read depth image"


# функция детектирования углов рамки
def frame_corners_detector():
    """
    return cords of corners
    """
    zeroes_image = np.zeros_like(image_binary)

    rgb_image_copy = rgb_image.copy()

    try:
        rgb_integrate = cv.bitwise_and(rgb_image, rgb_image, mask=image_binary)

        hsv_image = cv.cvtColor(rgb_integrate, cv.COLOR_BGR2HSV)

        # делаем бинаризацию картинки
        image_mask = cv.inRange(hsv_image, (46, 127, 0), (255, 255, 255))

        # находим контуры
        contours, hierarchy = cv.findContours(image_mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

        # сортируем контуры
        contours = sorted(contours, key=cv.contourArea, reverse=True)

        cv.drawContours(zeroes_image, contours[0], -1, 255, 5)

        # Уменьшаем контуры белых объектов - делаем 2 итераций
        zeroes_image = cv.erode(zeroes_image, None, iterations=2)

        # cv.imshow("zeroes_image", zeroes_image)

        # Show Features to Track
        gray = zeroes_image.copy()
        # ищем хорошие точки для трекинга в углах рамки
        corners = cv.goodFeaturesToTrack(gray, 4, 0.01, 10)
        corners = np.int0(corners)
        print corners


        # for i in corners:
        #     x, y = i.ravel()
        #     cv.circle(gray, (x, y), 3, 255, -1)
        #
        # cv.imshow('Gray', gray)
        #
        # # рисуем маркеры в найденых точках
        # for i in corners:
        #     cv.drawMarker(rgb_image_copy, tuple(i.ravel()), (0, 255, 0), markerType=cv.MARKER_TILTED_CROSS, thickness=2,
        #                   markerSize=50)
        #
        # cv.imshow("test", rgb_image_copy)

        if corners is not None:
            return corners

        else:
            return None

    except:
        print "detect frame error"


def main():
    global depth_frame, image_binary, rgb_image

    rospy.init_node("Frame_detector_node")

    # init subscriber
    rospy.Subscriber(depth_image_topic, Image, depth_image_cb)
    rospy.Subscriber(image_topic, Image, rgb_image_cb)


    while not rospy.is_shutdown():

        if image_binary is not None and depth_frame is not None:
            frame_corners_detector()

        else:
            print "image is not read"

        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break

# начало исполнения кода
if __name__ == "__main__":
    main()
