#!/usr/bin/env python
#coding=utf8

import rospy
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

camera_topic = "/mono_cam_forward/camera_mono/image_raw"
camera_file_port = "/dev/video2"

ros_image_forward = Image()
ros_image_down = Image()


# cap_forward = cv.VideoCapture(camera_file_port)# stereo elp >> /dev/video2, /dev/video4
# cap_forward.set(cv.CAP_PROP_FPS, 30) # Частота кадров
# cap_forward.set(cv.CAP_PROP_FRAME_WIDTH, 640) # Ширина кадров в видеопотоке.
# cap_forward.set(cv.CAP_PROP_FRAME_HEIGHT, 360) # Высота кадров в видеопотоке.

def img_cb_f(data):
    global ros_image_forward
    ros_image_forward = data
    print("fordsdasdasd")

def main():
    # global cv_image

    rospy.init_node('camera_frame_test')
    bridge = CvBridge()
    bridge_sec = CvBridge()

    rospy.Subscriber(camera_topic, Image, img_cb_f)

    while not rospy.is_shutdown():
        try:
            cv_img_down = bridge.imgmsg_to_cv2(ros_image_forward, "bgr8")
            # cv_img_forward = bridge_sec.imgmsg_to_cv2(ros_image_forward, "bgr8")
            # print("OK")
            cv.imshow("hren", cv_img_down)

            # извлекаем КРАСНЫЙ канал из кадра видеопотока
            r_channel = cv_img_down[:, :, 2]
            # создаем массив размером как r_channel
            binary_r = np.zeros_like(r_channel)
            # заносим соответствующие пиксели из r_channel, которые меньше 2 в массив binary_r со знаением 1
            binary_r[(r_channel < 100)] = 255

            # cv.imshow("r_channel", r_channel)
            # cv.imshow("binary_r", binary_r)

            # извлекаем из HLS канал отвечающий за яркость
            hls_img = cv.cvtColor(cv_img_down, cv.COLOR_BGR2HLS)
            s_channel = hls_img[:, :, 1]  # олд [:, :, 2]
            binary_s = np.zeros_like(s_channel)
            binary_s[(s_channel < 100)] = 255

            # cv.imshow("s_channel", s_channel)
            # cv.imshow("binary_s", binary_s)

            # объединяем два бинаризованного изображения в одно
            AllBinary = np.zeros_like(binary_r)
            AllBinary[((binary_r == 255) | (binary_s == 255))] = 255

            cv.imshow("AllBinary", AllBinary)
            # Фильтруем
            # # Уменьшаем контуры белых объектов - делаем две итерации
            # AllBinary = cv.erode(AllBinary, None, iterations=1)
            # # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем 5 итераций
            # AllBinary = cv.dilate(AllBinary, None, iterations=1)

            # cv.imshow("hren2", AllBinary)

        except:
            print("FAIL")

        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break


if __name__ == "__main__":
    main()
    cv.destroyAllWindows()
