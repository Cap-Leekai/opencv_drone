#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

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

cv_img = cv.imread('/home/leekay/catkin_ws/src/opencv_drone/images/frame_ang.png')

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

                r_channel = cv_img[ :, :, 2]
                # hsv = r_channel
                # cv.imshow("r_chnl", r_channel)
                hsv = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
                # cv.imshow('frame', hsv) # выводим картинку с камеры в формате HSV на экран

                # получаем значения задаваемые бегунками
                minb = cv.getTrackbarPos('minb', 'result')
                ming = cv.getTrackbarPos('ming', 'result')
                minr = cv.getTrackbarPos('minr', 'result')

                maxb = cv.getTrackbarPos('maxb', 'result')
                maxg = cv.getTrackbarPos('maxg', 'result')
                maxr = cv.getTrackbarPos('maxr', 'result')

                # делаем размытие картинки HSV
                hsv = cv.blur(hsv, (4, 4))
                # cv.imshow('Blur', hsv)

                # делаем бинаризацию картинки и пихаем её в переменную mask
                mask = cv.inRange(hsv, (minb, ming, minr), (maxb, maxg, maxr))
                # cv.imshow('mask', mask)

                # Уменьшаем контуры белых объектов - делаем две итерации
                maskEr = cv.erode(mask, None, iterations=2)
                # cv.imshow("Erode", maskEr)

                # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
                maskDi = cv.dilate(maskEr, None, iterations=2)
                # cv.imshow('Dilate', maskDi)

                # накладываем полученную маску на картинку с камеры переведённую в формат HSV
                result = cv.bitwise_and(cv_img, cv_img, mask = mask)
                # result = cv.resize(result, (500, 500))
                cv.imshow('result', result)

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