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


# инициализация топиков
cam_img_topic = "/iris_rplidar/usb_cam/image_raw"           # топик нижней камеры
alt_topic = "/drone/alt"                                            # топик текущей высоты
drone_pose_topic = "/mavros/local_position/pose"                    # топик текущей позиции
drone_goal_pose = "/goal_pose"


# переменные
goal_pose = Goal()
drone_pose = PoseStamped()
width_of_line = 0.2
drone_alt = Float32()


# функция преобразования локальных координат в глобальные координаты
def transform_cord(W, cords):

    X = (math.cos(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) + (math.sin(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.cos(W) - cords[1] * math.sin(W))
    Y = (math.sin(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) - (math.cos(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.sin(W) + cords[1] * math.cos(W))
    # print (W, X, Y)
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

    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size = 10)

    # основной цикл
    while not rospy.is_shutdown():
        try:
            if ros_img:
                cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
            else:
                print "Camera not read!"


            # извлекаем КРАСНЫЙ канал из кадра видеопотока
            r_channel = cv_img[:, :, 2]
            # создаем массив размером как r_channel
            binary_r = np.zeros_like(r_channel)
            # заносим соответствующие пиксели из r_channel, которые меньше 2 в массив binary_r со знаением 1
            binary_r[(r_channel == 0)] = 1

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
            histogram_right = np.sum(AllBinary[:, AllBinary.shape[1] // 2:], axis = 1)    # первым аргументом диапазона поставить -> AllBinary.shape[0] // 2
            histogram_left = np.sum(AllBinary[:, :AllBinary.shape[1] // 2], axis = 1)  # первым аргументом диапазона поставить -> AllBinary.shape[0] // 2

            midpoint_y = cv_img.shape[0] // 2
            midpoint_x = cv_img.shape[1] // 2

            IndWhitesColumnR = np.argmax(histogram_right) # [:histogram.shape[0]//2]
            IndWhitesColumnL = np.argmax(histogram_left)  # [:histogram.shape[0]//2]
            # OrientationIndWhitesColumnR = np.argmax(histogram[midpoint_y:]) + midpoint_y       # ????
            # OrientationIndWhitesColumnL = np.argmax(histogram[:midpoint_y])                  # ????
            allbinary = AllBinary.copy()
                                                                                                                                    # -(IndWhitesColumnL - midpoint_y)  это уже перевернутая ось игрек
            
            cv.line(allbinary, (0, allbinary.shape[0] // 2), (allbinary.shape[1], allbinary.shape[0] // 2), 200, 2)                 # рисуем статическую горизонталь
            cv.line(allbinary, (allbinary.shape[1] // 2, allbinary.shape[0] // 2), (allbinary.shape[1], IndWhitesColumnR), 180, 2)
            cv.line(allbinary, (0, IndWhitesColumnL), (allbinary.shape[1] // 2, allbinary.shape[0] // 2), 180, 2)


            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
            # print "Курс: %s     Смещение курса: %s" %(yaw, yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), cv_img.shape[1] // 2))     # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)

            # НАХОДИМ КОНТУРЫ
            contours, hierarchy = cv.findContours(AllBinary[:,AllBinary.shape[1] // 2 : AllBinary.shape[1]], cv.RETR_TREE, cv.CHAIN_APPROX_NONE)            #AllBinary.shape[1] // 2 : AllBinary.shape[1]]
            # сортируем контуры
            contours = sorted(contours, key = cv.contourArea, reverse=True)


            cords_of_rect = cv.boundingRect(contours[0])
            cords_of_rect = [cords_of_rect[0] + AllBinary.shape[1] // 2, cords_of_rect[1], cords_of_rect[2], cords_of_rect[3]]

            # вычленяем массив контуров из переменной contours и переинициализируем переменную contours
            cv.drawContours(cv_img, contours, -1, (0, 180, 255), 1)
            cv.rectangle(cv_img, (cords_of_rect[0], cords_of_rect[1]), (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]), (255, 0, 0), 1)                  # возвращает кортеж в формате  (x, y, w, h)
            cv.circle(cv_img, ((cords_of_rect[0] + cords_of_rect[2] // 2) , (cords_of_rect[1] + cords_of_rect[3] // 2) )  , 10, (0, 255, 0), -10)


            LIST = recalculation_cords(AllBinary)

            sm_pix_y = float(-(cords_of_rect[1] + cords_of_rect[3] // 2) + midpoint_y)          # вычисляем смещение от центра кадра в пикселях по y
            sm_pix_x = float((cords_of_rect[0] + cords_of_rect[2] // 2) - midpoint_x)           # вычисляем смещение от центра кадра в пикселях по x

            pix_on_meter = float((sum(LIST) // len(LIST))) // width_of_line

            correct_y = sm_pix_y / pix_on_meter
            correct_x = sm_pix_x / pix_on_meter

            # print correct_y
            print correct_x, correct_y

            cv.line(cv_img, (cv_img.shape[1], 0), (cv_img.shape[1], int(pix_on_meter)), (255, 0, 255), 10)       # отоюражаем линию масштаба - теоретически линия на кадре показывает МЕТР
            cv.imshow("Image", cv_img)

            # print IndWhitesColumnL - IndWhitesColumnR

            # if abs(IndWhitesColumnL - IndWhitesColumnR) < 30:
            #     goal_pose.pose.course = yaw

            # elif abs(IndWhitesColumnL - IndWhitesColumnR) > 30:
            #     goal_pose.pose.course = yaw + math.atan2(-(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), cv_img.shape[1] // 2)

            x_glob, y_glob = transform_cord(yaw, (correct_x, correct_y))
            goal_pose.pose.point.x = x_glob
            goal_pose.pose.point.y = y_glob           #drone_pose.pose.position.y +
            goal_pose.pose.point.z = 2.0
            goal_pose.pose.course = yaw + math.atan2(-(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), cv_img.shape[1] // 2)

            print "Смещение по y: %s" % correct_y
              # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)
            # print sm_pix_x, correct_y

            goal_pose_pub.publish(goal_pose)

            cv.imshow("test_test", allbinary)

            """
            
            Если разность IndWhitesColumnL и IndWhitesColumnR удовлетворяет дельта-окрестности     abs(IndWhitesColumnL - IndWhitesColumnR) < 0.4,       значит целевой курс БЛА соответствует нынешнему
            
            Но если IndWhitesColumn оба отличаются от констаны горизонтали, значит БЛА смещен относительно линии и его нужно скорректировать по координате Y
            
            """
        except:
            print "Main function has an error"


        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break

if __name__ == "__main__":

    print "_BEGIN_"
    main()
