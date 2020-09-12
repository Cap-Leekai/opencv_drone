#!/usr/bin/env python
#coding=utf8


import cv2 as cv
import rospy
import numpy as np
import math
import tf


from cv_bridge import CvBridge
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Quaternion
from drone_msgs.msg import Goal
from sensor_msgs.msg import Image
from opencv_drone.msg import frame_detect


# инициализация топиков
cam_img_topic = "/iris_rplidar/usb_cam/image_raw"                   # топик нижней камеры
alt_topic = "/drone/alt"                                            # топик текущей высоты
drone_pose_topic = "/mavros/local_position/pose"                    # топик текущей позиции
drone_goal_pose = "/goal_pose"
frame_detect_topic = "/frame_detector"


# переменные
goal_pose = Goal()
drone_pose = PoseStamped()
width_of_line = 0.2
drone_alt = Float32()
ros_img = None
detect_frame_flag = False

def detect_frame_cb(data):
    global detect_frame_flag
    detect_frame_flag = data.detect_frame
    # if data.detect_frame:
    #     detect_frame_flag = True
    #     # print "FRAME!!!!!"
    # else:
    #     detect_frame_flag = False
    #     print "NOT FRAME!!!!"


# функция преобразования локальных координат в глобальные координаты
def transform_cord(W, cords):

    X = (math.cos(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) + (math.sin(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.cos(W) - cords[1] * math.sin(W))
    Y = (math.sin(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) - (math.cos(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.sin(W) + cords[1] * math.cos(W))
    return X, Y


# функция вычисления количества пикселей на метр
def recalculation_cords(AllBinary):

    LIST = []
    for i in np.arange(0, AllBinary.shape[1]):
        LIST.append(np.sum(AllBinary[:, i] == 255))
    return LIST


# функция считывания текущего положения дрона
def drone_pose_cb(data):
    global drone_pose, quaternion
    drone_pose = data
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)


# функция считывания текущей высоты
def drone_alt_cb(data):
    global drone_alt
    drone_alt = data.data


# колбэк для считывания картинки из ROS
def cam_img_cb(data):
    global ros_img
    ros_img = data


# главная функция
def main():
    rospy.init_node('fly_by_line_node')
    bridge = CvBridge()


    # init subscribers
    rospy.Subscriber(cam_img_topic, Image, cam_img_cb)
    rospy.Subscriber(drone_pose_topic, PoseStamped, drone_pose_cb)
    rospy.Subscriber(alt_topic, Float32, drone_alt_cb)
    rospy.Subscriber(frame_detect_topic, frame_detect, detect_frame_cb)

    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size = 10)

    print "_BEGIN_"

    # основной цикл
    while not rospy.is_shutdown():
        try:
            while True:
                if ros_img:
                    cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
                    break
                else:
                    print "Camera not read!"

            if not detect_frame_flag:
                # извлекаем КРАСНЫЙ канал из кадра видеопотока
                r_channel = cv_img[:, :, 2]
                # создаем массив размером как r_channel
                binary_r = np.zeros_like(r_channel)
                # заносим соответствующие пиксели из r_channel, которые меньше 2 в массив binary_r со знаением 1
                binary_r[(r_channel == 0)] = 1

                # извлекаем из HLS канал отвечающий за яркость
                hls_img = cv.cvtColor(cv_img, cv.COLOR_BGR2HLS)
                s_channel = hls_img[:, :, 1]            # олд [:, :, 2]
                binary_s = np.zeros_like(s_channel)
                binary_s[(s_channel == 0)] = 1

                # объединяем два бинаризованного изображения в одно
                AllBinary = np.zeros_like(binary_r)
                AllBinary[((binary_r == 1)|(binary_s == 1))] = 255

                # Фильтруем
                # Уменьшаем контуры белых объектов - делаем две итерации
                AllBinary = cv.erode(AllBinary, None, iterations = 5)
                # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем 5 итераций
                AllBinary = cv.dilate(AllBinary, None, iterations = 5)

                # находим сумму всех элементов каждого столбца массива AllBinary в диапазоне от AllBinary.shape[0] // 2 до AllBinary.shape[0]
                histogram_right = np.sum(AllBinary[:, AllBinary.shape[1] // 2: ], axis = 1)
                histogram_left = np.sum(AllBinary[:, :AllBinary.shape[1] // 2], axis = 1)

                # найдём координаты центра кадра
                midpoint_y = cv_img.shape[0] // 2
                midpoint_x = cv_img.shape[1] // 2

                IndWhitesColumnR = np.argmax(histogram_right) # [:histogram.shape[0]//2]
                IndWhitesColumnL = np.argmax(histogram_left)  # [:histogram.shape[0]//2]

                allbinary_copy = AllBinary.copy()

                cv.line(allbinary_copy, (0, allbinary_copy.shape[0] // 2), (allbinary_copy.shape[1], allbinary_copy.shape[0] // 2), 200, 2)                 # рисуем статическую горизонталь

                cv.line(allbinary_copy, (allbinary_copy.shape[1] // 2, allbinary_copy.shape[0] // 2), (allbinary_copy.shape[1], IndWhitesColumnR), 180, 2)
                cv.line(allbinary_copy, (0, IndWhitesColumnL), (allbinary_copy.shape[1] // 2, allbinary_copy.shape[0] // 2), 180, 2)


                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
                # print "Курс: %s     Смещение курса: %s" %(yaw, yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), cv_img.shape[1] // 2))     # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)

                # НАХОДИМ КОНТУРЫ
                contours, hierarchy = cv.findContours(AllBinary[:,AllBinary.shape[1] // 2 : AllBinary.shape[1]], cv.RETR_TREE, cv.CHAIN_APPROX_NONE)            # AllBinary[:,AllBinary.shape[1] // 2 : AllBinary.shape[1]]

                if len(contours):
                    # сортируем контуры
                    contours = sorted(contours, key = cv.contourArea, reverse = True)

                    cords_of_rect = cv.boundingRect(contours[0])
                    cords_of_rect = [cords_of_rect[0] + AllBinary.shape[1] // 2, cords_of_rect[1], cords_of_rect[2], cords_of_rect[3]]

                    # вычленяем массив контуров из переменной contours
                    # cv.drawContours(cv_img, contours, -1, (0, 180, 255), 1)
                    cv.rectangle(cv_img, (cords_of_rect[0], cords_of_rect[1]), (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]), (255, 0, 0), 1)                  # возвращает кортеж в формате  (x, y, w, h)
                    cv.circle(cv_img, ((cords_of_rect[0] + cords_of_rect[2] // 2) , (cords_of_rect[1] + cords_of_rect[3] // 2) ), 10, (0, 255, 0), -10)

                    LIST = recalculation_cords(AllBinary)

                    sm_pix_y = float(-(cords_of_rect[1] + cords_of_rect[3] // 2) + midpoint_y)          # вычисляем смещение от центра кадра в пикселях по y
                    sm_pix_x = float((cords_of_rect[0] + cords_of_rect[2] // 2) - midpoint_x)           # вычисляем смещение от центра кадра в пикселях по x

                    # находим коэффициент пиксель на метр
                    pixel_on_meter = float((sum(LIST) // len(LIST))) // width_of_line

                    # находим координаты целевой точки в локальной системе координат
                    correct_y = sm_pix_y / pixel_on_meter
                    correct_x = sm_pix_x / pixel_on_meter

                    # отображаем линию масштаба - теоретически линия на кадре показывает МЕТР
                    cv.line(cv_img, (cv_img.shape[1], 0), (cv_img.shape[1], int(pixel_on_meter)), (255, 0, 255), 10)
                    # cv.imshow("Image", cv_img)

                    # переводим кокальные координаты целевой точки в глобальные
                    x_glob, y_glob = transform_cord(yaw, (correct_x, correct_y))

                    goal_pose.pose.point.x = x_glob
                    goal_pose.pose.point.y = y_glob

                    if drone_alt > 1.2:
                        goal_pose.pose.point.z = drone_alt
                    elif drone_alt < 1.3:
                        goal_pose.pose.point.z = 1.2


                    # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)
                    goal_pose.pose.course = yaw + math.atan2(-(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), cv_img.shape[1] // 2)
                    goal_pose_pub.publish(goal_pose)
                    #
                    # cv.imshow("test_test", allbinary_copy)
                else:
                    print "Need line"
            else:
                print "STOP!"

        except:
            print "Main function has an error"

        # проверяем была ли нажата кнопка esc
        if cv.waitKey(1) == 27:
            break

if __name__ == "__main__":
    main()
