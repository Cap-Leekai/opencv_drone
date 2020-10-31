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

cam_img_topic = "/d400/color/image_raw"                                   # топик нижней камеры
cam_down_img_topic = "/mono_cam_forward/camera_mono/image_raw"

drone_pose_topic = "/mavros/local_position/pose"                          # топик текущей позиции
drone_goal_pose = "/goal_pose"
frame_detect_topic = "/frame_detector"

# переменные
goal_pose = Goal()
drone_pose = PoseStamped()

image_width_px = 1280
image_height_px = 720

cut_tr = 170
h_trapeze = 50

image_size = [image_height_px, image_width_px]
width_of_line = 0.2
x_forvard = 1.8

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

limit_f_channel = 150
limit_d_channel = 80

ros_img = None
ros_img_down = None
quaternion = None

view_result_flag = True
detect_frame_flag = False

trapeze_cords = np.float32([[0, image_height_px], [image_width_px, image_height_px], [image_size[1] - cut_tr, image_size[0] - h_trapeze],  [cut_tr, image_size[0] - h_trapeze]])
trapeze_cords_draw = np.array(trapeze_cords, dtype=np.int32)

reshape_trapeze_cords = np.float32([[0, image_size[1]], [image_size[1], image_size[0]], [image_size[1], 0], [0, 0]])


# берем конфигурацию основных переменных из сервера параметров ROS
def get_params_server():
    global view_result_flag, image_width_px, image_height_px, limit_f_channel, maxb, maxg, maxr, h_trapeze, cut_tr, cam_img_topic, cam_down_img_topic, x_forvard, minb_down, ming_down, minr_down, maxb_down, maxg_down, maxr_down, limit_d_channel

    cam_img_topic = rospy.get_param('~cam_img_topic', cam_img_topic)
    cam_down_img_topic = rospy.get_param('~cam_down_img_topic', cam_down_img_topic)

    maxb = rospy.get_param('~maxb', maxb)
    maxg = rospy.get_param('~maxg', maxg)
    maxr = rospy.get_param('~maxr', maxr)

    minb_down = rospy.get_param('~minb_down', minb_down)
    ming_down = rospy.get_param('~ming_down', ming_down)
    minr_down = rospy.get_param('~minr_down', minr_down)

    maxb_down = rospy.get_param('~maxb_down', maxb_down)
    maxg_down = rospy.get_param('~maxg_down', maxg_down)
    maxr_down = rospy.get_param('~maxr_down', maxr_down)

    limit_d_channel = rospy.get_param('~limit_d_channel', limit_d_channel)
    limit_f_channel = rospy.get_param('~limit_f_channel', limit_f_channel)

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
def cam_down_img_cb(data):
    global ros_img_down
    ros_img_down = data


# колбэк для считывания картинки из ROS
def cam_img_cb(data):
    global ros_img
    ros_img = data


def detect_frame_cb(data):
    global detect_frame_flag
    detect_frame_flag = data.detect_frame


# главная функция
def main():
    rospy.init_node('fly_by_line_node')
    bridge = CvBridge()

    get_params_server()

    # init subscribers
    rospy.Subscriber(cam_img_topic, Image, cam_img_cb)
    # rospy.Subscriber(cam_down_img_topic, Image, cam_down_img_cb)

    rospy.Subscriber(drone_pose_topic, PoseStamped, drone_pose_cb)
    rospy.Subscriber(frame_detect_topic, frame_detect, detect_frame_cb)


    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size = 10)
    hz = rospy.Rate(10)

    rospy.loginfo("_BEGIN_")

    # основной цикл
    while not rospy.is_shutdown():
        if ros_img:
            cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")

            cv.imshow("cv_img", cv_img)

        else:
            rospy.loginfo_throttle(6, "Camera not read!")
            continue


        f_channel = cv_img[:, :, 2]
        # создаем массив размером как r_channel
        binary_f = np.zeros_like(f_channel)
        # заносим соответствующие пиксели из r_channel, которые меньше 2 в массив binary_r со знаением 1
        binary_f[(f_channel < limit_f_channel)] = 255
        cv.imshow("hh", binary_f)

        # Фильтруем
        # Уменьшаем контуры белых объектов - делаем две итерации
        # AllBinary = cv.erode(binary_f, None, iterations = 1)
        # # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем 5 итераций
        # AllBinary = cv.dilate(AllBinary, None, iterations = 1)

        AllBinary_trapeze = binary_f.copy()

        # рисуем трапецию на AllBinary
        AllBinary_trapeze = cv.polylines(AllBinary_trapeze, [trapeze_cords_draw], True, 255)

        cv.imshow("trapeze", AllBinary_trapeze)

        # считаем матрицу преобразования
        M = cv.getPerspectiveTransform(trapeze_cords, reshape_trapeze_cords)

        # преобразуем трапецию в полноценный кадр
        warped = cv.warpPerspective(binary_f, M, (image_size[1], image_size[0]), flags=cv.INTER_LINEAR)

        cv.imshow("warped", warped)

        # находим сумму всех элементов каждого столбца массива AllBinary в диапазоне от AllBinary.shape[0] // 2 до AllBinary.shape[0]
        histogram_curse = np.sum(warped[:, :], axis = 0)

        # найдём координаты центра кадра
        midpoint_y = cv_img.shape[0] // 2
        midpoint_x = cv_img.shape[1] // 2

        IndWhitesColumnU = np.argmax(histogram_curse) # [:histogram.shape[0]//2]

        if IndWhitesColumnU == 0:
            rospy.loginfo_throttle(5, "Line lost!")
            continue
        #
        #
        #
        #
        #     allbinary_copy = warped.copy()
        #
        #     if view_result_flag:
        #         # рисуем горизонталь
        #         cv.line(allbinary_copy, (0, allbinary_copy.shape[0] // 2), (allbinary_copy.shape[1], allbinary_copy.shape[0] // 2), 200, 2)                 # рисуем статическую горизонталь
        #         # рисуем линии смещения пути в кадре,
        #         cv.line(allbinary_copy, (allbinary_copy.shape[1] // 2, allbinary_copy.shape[0]), (IndWhitesColumnU, 0), 180, 2)
        #
        #     if quaternion is not None:
        #         (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        #     else:
        #         continue
        #
        #     print "Курс: %s     Смещение курса: %s" %(yaw, yaw - math.atan2((IndWhitesColumnU - midpoint_x), cv_img.shape[0]))     # целевой курс в goal_pose -> yaw + math.atan2( -(IndWhitesColumnR - midpoint_y) - (-(IndWhitesColumnL - midpoint_y)), 320)
        #
        #     print(math.atan2((IndWhitesColumnU - midpoint_x), allbinary_copy.shape[0] // 2))
        #     goal_pose.pose.course = yaw - math.atan2((IndWhitesColumnU - midpoint_x), cv_img.shape[0])
        #
        #     goal_pose_pub.publish(goal_pose)
        #
        #     if view_result_flag:
        #         cv.imshow("test1", warped)
        #         cv.imshow("test2", AllBinary_trapeze)
        #         cv.imshow("test3", allbinary_copy)
                # cv.imshow("test4", allbinary_copy_down)

            # hz.sleep()
            # проверяем была ли нажата кнопка esc
        if cv.waitKey(1) == 27:
            break
        # else:
        #     rospy.loginfo("Stop!")


if __name__ == "__main__":
    main()
