#!/usr/bin/env python
#coding=utf8


import cv2 as cv
import rospy
import numpy as np
import math
import tf


from cv_bridge import CvBridge
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, TwistStamped
from drone_msgs.msg import Goal
from sensor_msgs.msg import Image
from opencv_drone.msg import frame_detect


# инициализация топиков
cam_img_topic = "/r200/image_raw"                                   # топик нижней камеры
cam_down_img_topic = "/iris_rplidar/usb_cam/image_raw"
drone_pose_topic = "/mavros/local_position/pose"                    # топик текущей позиции
drone_goal_pose = "/goal_pose"

# переменные
goal_pose = Goal()
drone_pose = PoseStamped()

image_width_px = 640
image_height_px = 480

cut_tr = 100
h_trapeze = 60

image_size = [image_height_px, image_width_px]
width_of_line = 0.2
x_forvard = 3.0

minb = 0
ming = 0
minr = 0

maxb = 0
maxg = 0
maxr = 0

minb_down = 0
ming_down = 0
minr_down = 0

maxb_down = 100
maxg_down = 70
maxr_down = 64

limit_r_channel = 100

ros_img = None
ros_img_down = None
quaternion = None

view_result_flag = True

trapeze_cords = np.float32([[0, image_height_px], [image_width_px, image_height_px], [540, image_size[0] - h_trapeze],  [100, image_size[0] - h_trapeze]])
trapeze_cords_draw = np.array(trapeze_cords, dtype=np.int32)

reshape_trapeze_cords = np.float32([[0, image_size[1]], [image_size[1], image_size[0]], [image_size[1], 0], [0, 0]])


# берем конфигурацию основных переменных из сервера параметров ROS
def get_params_server():
    global view_result_flag, image_width_px, image_height_px, maxb, maxg, maxr, h_trapeze, cut_tr, cam_img_topic, x_forvard, minb_down, ming_down, minr_down, maxb_down, maxg_down, maxr_down, limit_r_channel
    cam_img_topic = rospy.get_param('~cam_img_topic', cam_img_topic)

    maxb = rospy.get_param('~maxb', maxb)
    maxg = rospy.get_param('~maxg', maxg)
    maxr = rospy.get_param('~maxr', maxr)

    minb_down = rospy.get_param('~minb_down', minb_down)
    ming_down = rospy.get_param('~ming_down', ming_down)
    minr_down = rospy.get_param('~minr_down', minr_down)

    maxb_down = rospy.get_param('~maxb_down', maxb_down)
    maxg_down = rospy.get_param('~maxg_down', maxg_down)
    maxr_down = rospy.get_param('~maxr_down', maxr_down)

    limit_r_channel = rospy.get_param('~limit_r_channel', limit_r_channel)

    h_trapeze = rospy.get_param('~h_trapeze', h_trapeze)
    cut_tr = rospy.get_param('~cut_tr', cut_tr)

    image_width_px = rospy.get_param('~image_width_px', image_width_px)
    image_height_px = rospy.get_param('~image_height_px', image_height_px)

    view_result_flag = rospy.get_param('~view_result_flag', view_result_flag)

    x_forvard = rospy.get_param('~x_forvard', x_forvard)

    rospy.loginfo("init params done")


# функция преобразования локальных координат в глобальные координаты
def transform_cord(W, cords):

    X = (math.cos(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) + (math.sin(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.cos(W) - cords[1] * math.sin(W))
    Y = (math.sin(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) - (math.cos(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.sin(W) + cords[1] * math.cos(W))
    return X, Y


# функция вычисления количества пикселей на метр
def recalculation_cords(AllBinary):
    LIST = []
    for i in np.arange(0, AllBinary.shape[0]):
        LIST.append(np.sum(AllBinary[i, :] == 255))
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


# колбэк для считывания картинки из ROS
def cam_down_img_cb(data):
    global ros_img_down
    ros_img_down = data


# главная функция
def main():
    rospy.init_node('fly_by_line_node')
    bridge = CvBridge()

    get_params_server()

    # init subscribers
    rospy.Subscriber(cam_img_topic, Image, cam_img_cb)
    rospy.Subscriber(cam_down_img_topic, Image, cam_down_img_cb)

    rospy.Subscriber(drone_pose_topic, PoseStamped, drone_pose_cb)

    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size = 10)
    hz = rospy.Rate(10)

    rospy.loginfo("_BEGIN_")
    # основной цикл
    while not rospy.is_shutdown():
        if ros_img and ros_img_down:
            cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
            cv_img_down = bridge.imgmsg_to_cv2(ros_img_down, "bgr8")
        else:
            rospy.loginfo_throttle(6, "Camera not read!")
            continue
        #####

        #cv_img -> bin_img



        AllBinary = cv.inRange(cv_img, (minb, ming, minr), (maxb, maxg, maxr))

        hls = cv.cvtColor(cv_img_down, cv.COLOR_BGR2HLS)
        # cv.imshow('frame', hsv) # выводим картинку с камеры в формате HSV на экран
        r_channel = hls[:, :, 1]
        binary_s = np.zeros_like(r_channel)
        binary_s[(r_channel < limit_r_channel)] = 255

        # Уменьшаем контуры белых объектов - делаем две итерации
        maskEr = cv.erode(binary_s, None, iterations=1)
        # cv.imshow("Erode", maskEr)

        # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
        AllBinary_down = cv.dilate(maskEr, None, iterations=1)
        # cv.imshow('Dilate', maskDi)

        # AllBinary_down = cv.inRange(cv_img_down, (minb_down, ming_down, minr_down), (maxb_down, maxg_down, maxr_down))

        #####
        # извлекаем КРАСНЫЙ канал из кадра видеопотока
        # r_channel = cv_img[:, :, 2]
        # # создаем массив размером как r_channel
        # binary_r = np.zeros_like(r_channel)
        # # заносим соответствующие пиксели из r_channel, которые меньше 2 в массив binary_r со знаением 1
        # binary_r[(r_channel == 0)] = 1
        #
        # # извлекаем из HLS канал отвечающий за яркость
        # hls_img = cv.cvtColor(cv_img, cv.COLOR_BGR2HLS)
        # s_channel = hls_img[:, :, 1]            # олд [:, :, 2]
        # binary_s = np.zeros_like(s_channel)
        # binary_s[(s_channel == 0)] = 1
        #
        # # объединяем два бинаризованного изображения в одно
        # AllBinary = np.zeros_like(binary_r)
        # AllBinary[((binary_r == 1)|(binary_s == 1))] = 255

        # Фильтруем
        # Уменьшаем контуры белых объектов - делаем две итерации
        AllBinary = cv.erode(AllBinary, None, iterations = 1)
        # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем 5 итераций
        AllBinary = cv.dilate(AllBinary, None, iterations = 1)

        AllBinary_trapeze = AllBinary.copy()

        # рисуем трапецию на AllBinary
        AllBinary_trapeze = cv.polylines(AllBinary_trapeze, [trapeze_cords_draw], True, 255)

        # считаем матрицу преобразования
        M = cv.getPerspectiveTransform(trapeze_cords, reshape_trapeze_cords)

        # преобразуем трапецию в полноценный кадр
        warped = cv.warpPerspective(AllBinary, M, (image_size[1], image_size[0]), flags=cv.INTER_LINEAR)

        # находим сумму всех элементов каждого столбца массива AllBinary в диапазоне от AllBinary.shape[0] // 2 до AllBinary.shape[0]
        histogram_curse = np.sum(warped[:, :], axis = 0)

        # найдём координаты центра кадра
        midpoint_y = cv_img.shape[0] // 2
        midpoint_x = cv_img.shape[1] // 2

        IndWhitesColumnU = np.argmax(histogram_curse) # [:histogram.shape[0]//2]

        if IndWhitesColumnU == 0:
            rospy.loginfo_throttle(5, "Line lost!")
            continue

        # ___Приступаем к вычислению смещения коптера от линии___ #
        #******#
        # # НАХОДИМ КОНТУРЫ #
        # contours, hierarchy = cv.findContours(warped[240:, :], cv.RETR_TREE, cv.CHAIN_APPROX_NONE)           # [AllBinary.shape[0] // 2 + 200:, :]
        #
        # if len(contours):
        #     # сортируем контуры
        #     contours = sorted(contours, key = cv.contourArea, reverse = True)
        #     cords_of_rect = cv.boundingRect(contours[0]) #берем самый большой контур из массива контуров
        #     # cords_of_rect = [cords_of_rect[0] + AllBinary.shape[1] // 2 , cords_of_rect[1], cords_of_rect[2], cords_of_rect[3]]
        #
        #     if view_result_flag:
        #         cv_img_copy = cv_img.copy()
        #         # вычленяем массив контуров из переменной contours
        #         cv.drawContours(cv_img_copy, contours, -1, (0, 180, 255), 1)
        #         cv.rectangle(cv_img_copy, (cords_of_rect[0], cords_of_rect[1]), (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]), (255, 0, 0), 1)                  # возвращает кортеж в формате  (x, y, w, h)
        #         cv.circle(cv_img_copy, ((cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2)), 10, (0, 255, 0), -10)
        #         cv.imshow("Contours", cv_img_copy)
        #
        #     sm_pix_x = -float((cords_of_rect[0] + cords_of_rect[2] // 2) - midpoint_x)           # вычисляем смещение от центра кадра в пикселях по x
        #     rospy.loginfo("sm_pix_x: %s", sm_pix_x)
        #
        #     ####
        #     LIST = recalculation_cords(warped[warped.shape[0] // 2:, :])
        #
        #     # находим коэффициент пиксель на метр
        #     pixel_on_meter = float((sum(LIST) // len(LIST))) // width_of_line
        #     # rospy.loginfo("pixel_on_meter: %s" % pixel_on_meter)
        #     ####
        #     if pixel_on_meter == 0:
        #         continue
        #     correct_y = (sm_pix_x / pixel_on_meter)
        #     rospy.loginfo("correct_y: %s", correct_y)
        #******#

        #******#
        # Фильтруем
        # Уменьшаем контуры белых объектов - делаем две итерации
        AllBinary_down = cv.erode(AllBinary_down, None, iterations=5)
        # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем 5 итераций
        AllBinary_down = cv.dilate(AllBinary_down, None, iterations=5)

        # находим сумму всех элементов каждого столбца массива AllBinary в диапазоне от AllBinary.shape[0] // 2 до AllBinary.shape[0]
        histogram_right_down = np.sum(AllBinary_down[:, AllBinary_down.shape[1] // 2:], axis=1)
        histogram_left_down = np.sum(AllBinary_down[:, :AllBinary_down.shape[1] // 2], axis=1)

        # найдём координаты центра кадра
        midpoint_y_down = cv_img_down.shape[0] // 2
        midpoint_x_down = cv_img_down.shape[1] // 2

        IndWhitesColumnR = np.argmax(histogram_right_down)  # [:histogram.shape[0]//2]
        IndWhitesColumnL = np.argmax(histogram_left_down)  # [:histogram.shape[0]//2]

        allbinary_copy_down = AllBinary_down.copy()

        cv.line(allbinary_copy_down, (0, allbinary_copy_down.shape[0] // 2),
                (allbinary_copy_down.shape[1], allbinary_copy_down.shape[0] // 2), 200, 2)  # рисуем статическую горизонталь

        cv.line(allbinary_copy_down, (allbinary_copy_down.shape[1] // 2, allbinary_copy_down.shape[0] // 2),
                (allbinary_copy_down.shape[1], IndWhitesColumnR), 180, 2)
        cv.line(allbinary_copy_down, (0, IndWhitesColumnL), (allbinary_copy_down.shape[1] // 2, allbinary_copy_down.shape[0] // 2),
                180, 2)

        if quaternion is not None:
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        else:
            continue
        # print "Курс: %s     Смещение курса: %s" %(yaw, yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), cv_img.shape[1] // 2))     # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)

        # НАХОДИМ КОНТУРЫ
        contours, hierarchy = cv.findContours(AllBinary_down[:, AllBinary_down.shape[1] // 2: AllBinary_down.shape[1]],
                                              cv.RETR_TREE,
                                              cv.CHAIN_APPROX_NONE)  # AllBinary[:,AllBinary.shape[1] // 2 : AllBinary.shape[1]]     AllBinary.shape[1] // 2 : AllBinary.shape[1]

        if len(contours):
            # сортируем контуры
            contours = sorted(contours, key=cv.contourArea, reverse=True)

            cords_of_rect = cv.boundingRect(contours[0])
            cords_of_rect = [cords_of_rect[0] + AllBinary_down.shape[1] // 2, cords_of_rect[1], cords_of_rect[2],
                             cords_of_rect[3]]

            # вычленяем массив контуров из переменной contours
            # cv.drawContours(cv_img_down, contours, -1, (0, 180, 255), 1)
            cv.rectangle(cv_img_down, (cords_of_rect[0], cords_of_rect[1]),
                         (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]), (255, 0, 0),
                         1)  # возвращает кортеж в формате  (x, y, w, h)
            cv.circle(cv_img_down,
                      ((cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2)), 10,
                      (0, 255, 0), -10)

            LIST = recalculation_cords(AllBinary_down)

            sm_pix_y = float(-(cords_of_rect[1] + cords_of_rect[
                3] // 2) + midpoint_y)  # вычисляем смещение от центра кадра в пикселях по y
            sm_pix_x = float((cords_of_rect[0] + cords_of_rect[
                2] // 2) - midpoint_x)  # вычисляем смещение от центра кадра в пикселях по x

            # находим коэффициент пиксель на метр
            pixel_on_meter_down = float((sum(LIST) // len(LIST))) // width_of_line

            if pixel_on_meter_down == 0:
                continue

            # находим координаты целевой точки в локальной системе координат
            correct_y = (sm_pix_y / pixel_on_meter_down)
            correct_x = (sm_pix_x / pixel_on_meter_down) + 1.8

            rospy.loginfo("correct_y: %s" %correct_y)
        #******#


        allbinary_copy = warped.copy()

        if view_result_flag:
            # рисуем горизонталь
            cv.line(allbinary_copy, (0, allbinary_copy.shape[0] // 2), (allbinary_copy.shape[1], allbinary_copy.shape[0] // 2), 200, 2)                 # рисуем статическую горизонталь
            # рисуем линии смещения пути в кадре,
            cv.line(allbinary_copy, (allbinary_copy.shape[1] // 2, allbinary_copy.shape[0]), (IndWhitesColumnU, 0), 180, 2)

        if quaternion is not None:
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        else:
            continue

        # print "Курс: %s     Смещение курса: %s" %(yaw, yaw - math.atan2((IndWhitesColumnU - midpoint_x), cv_img.shape[0]))     # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)

        # print(math.atan2((IndWhitesColumnU - midpoint_x), allbinary_copy.shape[0] // 2))

        # переводим кокальные координаты целевой точки в глобальные
        x_glob, y_glob = transform_cord(yaw, (correct_x, correct_y))

        goal_pose.pose.point.x = x_glob
        goal_pose.pose.point.y = y_glob

        # if drone_alt > 1.2:
        goal_pose.pose.point.z = 1.56
        # elif drone_alt < 1.3:
        #     goal_pose.pose.point.z = 1.2

        # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)
        goal_pose.pose.course = yaw - math.atan2((IndWhitesColumnU - midpoint_x), cv_img.shape[0])

        goal_pose_pub.publish(goal_pose)

        if view_result_flag:
            cv.imshow("test1", warped)
            cv.imshow("test2", AllBinary_trapeze)
            cv.imshow("test3", allbinary_copy)
            cv.imshow("test4", allbinary_copy_down)




        hz.sleep()
        # проверяем была ли нажата кнопка esc
        if cv.waitKey(1) == 27:
            break

if __name__ == "__main__":
    main()
