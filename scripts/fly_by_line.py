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
drone_pose_topic = "/mavros/local_position/pose"                    # топик текущей позиции
drone_goal_pose = "/goal_pose"

# переменные
goal_pose = Goal()
drone_pose = PoseStamped()
image_size = [480, 640]
h_trapeze = 100
width_of_line = 0.2
ros_img = None
quaternion = None

view_result_flag = True

trapeze_cords = np.float32([[0, 480], [640, 480], [540, image_size[0] - h_trapeze],  [100, image_size[0] - h_trapeze]])
trapeze_cords_draw = np.array(trapeze_cords, dtype=np.int32)

reshape_trapeze_cords = np.float32([[0, image_size[1]], [image_size[1], image_size[0]], [image_size[1], 0], [0, 0]])


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


# главная функция
def main():
    rospy.init_node('fly_by_line_node')
    bridge = CvBridge()


    # init subscribers
    rospy.Subscriber(cam_img_topic, Image, cam_img_cb)
    rospy.Subscriber(drone_pose_topic, PoseStamped, drone_pose_cb)

    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size = 10)
    hz = rospy.Rate(10)

    rospy.loginfo("_BEGIN_")
    # основной цикл
    while not rospy.is_shutdown():
        if ros_img:
            cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
        else:
            rospy.loginfo_throttle(6, "Camera not read!")
            continue


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

        #___Приступаем к вычислению смещения коптера от линии___#

        # НАХОДИМ КОНТУРЫ
        contours, hierarchy = cv.findContours(warped, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)           # [AllBinary.shape[0] // 2 + 200:, :]

        if len(contours):
            # сортируем контуры
            contours = sorted(contours, key = cv.contourArea, reverse = True)
            cords_of_rect = cv.boundingRect(contours[0]) # берем самый большой контур из массива контуров
            # cords_of_rect = [cords_of_rect[0] + AllBinary.shape[1] // 2 , cords_of_rect[1], cords_of_rect[2], cords_of_rect[3]]

            if view_result_flag:
                cv_img_copy = cv_img.copy()
                # вычленяем массив контуров из переменной contours
                cv.drawContours(cv_img_copy, contours, -1, (0, 180, 255), 1)
                cv.rectangle(cv_img_copy, (cords_of_rect[0], cords_of_rect[1]), (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]), (255, 0, 0), 1)                  # возвращает кортеж в формате  (x, y, w, h)
                cv.circle(cv_img_copy, ((cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2)), 10, (0, 255, 0), -10)
                cv.imshow("Contours", cv_img_copy)

            sm_pix_x = float((cords_of_rect[0] + cords_of_rect[2] // 2) - midpoint_x)           # вычисляем смещение от центра кадра в пикселях по x
            rospy.loginfo("sm_pix_x: %s", sm_pix_x)

            ####
            LIST = recalculation_cords(warped[warped.shape[0] // 2:, :])

            # находим коэффициент пиксель на метр
            pixel_on_meter = float((sum(LIST) // len(LIST))) // width_of_line
            # rospy.loginfo("pixel_on_meter: %s" % pixel_on_meter)
            ####
            if pixel_on_meter == 0:
                continue
            correct_y = (sm_pix_x / pixel_on_meter)
            rospy.loginfo("correct_y: %s", correct_y)

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
        x_glob, y_glob = transform_cord(yaw, (3.0, correct_y))

        goal_pose.pose.point.x = x_glob
        goal_pose.pose.point.y = y_glob

        # if drone_alt > 1.2:
        goal_pose.pose.point.z = 2.0
        # elif drone_alt < 1.3:
        #     goal_pose.pose.point.z = 1.2


        # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)
        goal_pose.pose.course = yaw - math.atan2((IndWhitesColumnU - midpoint_x), cv_img.shape[0])
        goal_pose_pub.publish(goal_pose)

        if view_result_flag:
            cv.imshow("test1", warped)
            cv.imshow("test2", AllBinary_trapeze)
            cv.imshow("test3", allbinary_copy)


        #
        #     # находим сумму всех элементов каждого столбца массива AllBinary в диапазоне от AllBinary.shape[0] // 2 до AllBinary.shape[0]
        #     histogram_right = np.sum(AllBinary[:, AllBinary.shape[1] // 2: ], axis = 1)
        #     histogram_left = np.sum(AllBinary[:, :AllBinary.shape[1] // 2], axis = 1)
        #
        #     # найдём координаты центра кадра
        #     midpoint_y = cv_img.shape[0] // 2
        #     midpoint_x = cv_img.shape[1] // 2
        #
        #     IndWhitesColumnR = np.argmax(histogram_right) # [:histogram.shape[0]//2]
        #     IndWhitesColumnL = np.argmax(histogram_left)  # [:histogram.shape[0]//2]
        #
        #     allbinary_copy = AllBinary.copy()
        #
        #     cv.line(allbinary_copy, (0, allbinary_copy.shape[0] // 2), (allbinary_copy.shape[1], allbinary_copy.shape[0] // 2), 200, 2)                 # рисуем статическую горизонталь
        #
        #     cv.line(allbinary_copy, (allbinary_copy.shape[1] // 2, allbinary_copy.shape[0] // 2), (allbinary_copy.shape[1], IndWhitesColumnR), 180, 2)
        #     cv.line(allbinary_copy, (0, IndWhitesColumnL), (allbinary_copy.shape[1] // 2, allbinary_copy.shape[0] // 2), 180, 2)
        #
        #
        #
        #     # НАХОДИМ КОНТУРЫ
        #     contours, hierarchy = cv.findContours(AllBinary[:,AllBinary.shape[1] // 2 : AllBinary.shape[1]], cv.RETR_TREE, cv.CHAIN_APPROX_NONE)            # AllBinary[:,AllBinary.shape[1] // 2 : AllBinary.shape[1]]     AllBinary.shape[1] // 2 : AllBinary.shape[1]
        #
        #     if len(contours):
        #         # сортируем контуры
        #         contours = sorted(contours, key = cv.contourArea, reverse = True)
        #
        #         cords_of_rect = cv.boundingRect(contours[0])
        #         cords_of_rect = [cords_of_rect[0] + AllBinary.shape[1] // 2 , cords_of_rect[1], cords_of_rect[2], cords_of_rect[3]]
        #
        #         # вычленяем массив контуров из переменной contours
        #         # cv.drawContours(cv_img, contours, -1, (0, 180, 255), 1)
        #         cv.rectangle(cv_img, (cords_of_rect[0], cords_of_rect[1]), (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]), (255, 0, 0), 1)                  # возвращает кортеж в формате  (x, y, w, h)
        #         cv.circle(cv_img, ((cords_of_rect[0] + cords_of_rect[2] // 2) , (cords_of_rect[1] + cords_of_rect[3] // 2) ), 10, (0, 255, 0), -10)
        #
        #         LIST = recalculation_cords(AllBinary)
        #
        #         sm_pix_y = float(-(cords_of_rect[1] + cords_of_rect[3] // 2) + midpoint_y)          # вычисляем смещение от центра кадра в пикселях по y
        #         sm_pix_x = float((cords_of_rect[0] + cords_of_rect[2] // 2) - midpoint_x)           # вычисляем смещение от центра кадра в пикселях по x
        #
        #         # находим коэффициент пиксель на метр
        #         pixel_on_meter = float((sum(LIST) // len(LIST))) // width_of_line
        #
        #         # находим координаты целевой точки в локальной системе координат
        #         correct_y = (sm_pix_y / pixel_on_meter) + 1.0
        #         correct_x = (sm_pix_x / pixel_on_meter) + 1.0
        #
        #         # отображаем линию масштаба - теоретически линия на кадре показывает МЕТР
        #         cv.line(cv_img, (cv_img.shape[1], 0), (cv_img.shape[1], int(pixel_on_meter)), (255, 0, 255), 10)
        #         cv.imshow("Image", cv_img)
        #
        #         # переводим кокальные координаты целевой точки в глобальные
        #         x_glob, y_glob = transform_cord(yaw, (correct_x, correct_y))
        #
        #         goal_pose.pose.point.x = x_glob
        #         goal_pose.pose.point.y = y_glob
        #
        #         if drone_alt > 1.2:
        #             goal_pose.pose.point.z = drone_pose.pose.position.z
        #         elif drone_alt < 1.3:
        #             goal_pose.pose.point.z = 1.2
        #
        #
        #         # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)
        #         goal_pose.pose.course = yaw + math.atan2(-(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), cv_img.shape[1] // 2)
        #         # goal_pose_pub.publish(goal_pose)
        #
        #         # cv.imshow("test_test", allbinary_copy)
        #     else:
        #         rospy.loginfo("Need line")
        # else:
        #     rospy.loginfo("STOP!")
        #
        # except:
        #     print "Main function has an error"
        hz.sleep()
        # проверяем была ли нажата кнопка esc
        if cv.waitKey(1) == 27:
            break

if __name__ == "__main__":
    main()
