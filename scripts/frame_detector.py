#!/usr/bin/env python
#coding=utf8


import cv2 as cv
import numpy as np
import rospy
import rospkg
import os
import tf
import math
import time

from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from opencv_drone.msg import frame_detect
from std_msgs.msg import Float32
from drone_msgs.msg import Goal
from geometry_msgs.msg import PoseStamped, Quaternion, Point

depth_image_topic = "/r200/depth/image_raw"
image_topic = "/r200/image_raw"
frame_detect_topic = "/frame_detector"
alt_topic = "/drone/alt"                                            # топик текущей высоты
drone_pose_topic = "/mavros/local_position/pose"
drone_goal_pose = "/goal_pose"

marker_publisher = None
depth_frame = None
image_binary = None
rgb_image = None
corners = None

l = 1.5                # Плечо рамки в метрах
height_of_drone = 0.6  # Высота дрона
width_of_drone = 0.8   # Ширина дрона

frame_detect_flag = frame_detect()
goal_pose = Goal()
drone_pose = PoseStamped()
drone_alt = Float32()
flag = True
window_detect_flag = False


# классы для функции пролета в рамку
class goal:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class pointsFrame:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class pointsDrone:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


# функция считывания текущего положения дрона
def drone_pose_cb(data):
    global drone_pose, quaternion, roll, pitch, yaw
    drone_pose = data
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

# функция считывания текущей высоты
def drone_alt_cb(data):
    global drone_alt
    drone_alt = data.data


# функция преобразования локальных координат в глобальные координаты
def transform_cord(W, cords):

    X = (math.cos(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) + (math.sin(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.cos(W) - cords[1] * math.sin(W))
    Y = (math.sin(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) - (math.cos(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.sin(W) + cords[1] * math.cos(W))
    return X, Y


# функция трансформации координат в трехмерной системе координат
def transform_cords_3D(X, Y, Z, roll, pitch, yaw, goal_):

    # Matrix_R = np.array([
    #     [X * (math.cos(roll) * math.cos(yaw) - math.sin(roll) * math.cos(pitch) * math.sin(yaw)) + goal_.x * (math.cos(roll) * math.cos(yaw) - math.sin(roll) * math.cos(pitch) * math.sin(yaw)) - Y * (math.cos(roll) * math.sin(yaw) + math.sin(roll) * math.cos(pitch) * math.cos(yaw)) - goal_.y * (math.cos(roll) * math.sin(yaw) + math.sin(roll) * math.cos(pitch) * math.cos(yaw)) + Z * math.sin(roll) * math.sin(pitch) + goal_.z * math.sin(roll) * math.sin(pitch)],
    #     [X * (math.sin(roll) * math.cos(yaw) - math.cos(roll) * math.cos(pitch) * math.sin(yaw)) + goal_.x * (math.sin(roll) * math.cos(yaw) + math.cos(roll) * math.cos(pitch) * math.sin(yaw)) + Y * (math.sin(roll) * math.cos(yaw) + math.cos(roll) * math.cos(pitch) * math.sin(yaw)) - goal_.y * (math.sin(roll) * math.sin(yaw) + math.cos(roll) * math.cos(pitch) * math.cos(yaw)) - Z * math.cos(roll) * math.sin(pitch) + goal_.z * math.cos(roll) * math.sin(pitch)],
    #     [Z * math.cos(pitch) + goal_.z * math.cos(pitch) + X * math.sin(pitch) * math.sin(yaw) + goal_.x * math.sin(pitch) * math.sin(yaw) + Y * math.sin(pitch)*math.cos(yaw) + goal_.y * math.sin(pitch) * math.cos(yaw)]
    # ])

    glob_cords = np.array([X, Y, Z])

    local_cords = np.array([goal_.x, goal_.y, goal_.z])

    transpose_cord = local_cords.reshape(3, 1)

    matrix_R = np.array([[math.cos(roll) * math.cos(yaw) - math.sin(roll) * math.cos(pitch) * math.sin(yaw), - math.cos(roll) * math.sin(yaw) - math.sin(roll) * math.cos(pitch) * math.cos(yaw), math.sin(roll) * math.sin(pitch)],
                         [math.sin(roll) * math.cos(yaw) + math.cos(roll) * math.cos(pitch) * math.sin(yaw), - math.sin(roll) * math.sin(yaw) + math.cos(roll) * math.cos(pitch) * math.cos(yaw), - math.cos(roll) * math.sin(pitch)],
                         [math.sin(pitch) * math.sin(yaw), math.sin(pitch) * math.cos(yaw), math.cos(pitch)]])

    glob_cords_of_point = np.dot(matrix_R, local_cords)

    glob_cords_of_point = [glob_cords_of_point[0] + X, glob_cords_of_point[1] + Y, glob_cords_of_point[2] + Z]

    print "glob_cords -> ", glob_cords_of_point
    return glob_cords_of_point

# callback считывания картинки с realsence в rgb
def rgb_image_cb(data):
    global rgb_image
    try:
        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")

    except:
        print "Error read rgb_image"
        rgb_image = None


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
        image_binary[(depth_frame < 4.9) & (depth_frame > 1.)] = 255

        image_binary = np.uint8(image_binary)

    except:
        print "Error read depth image"
        depth_image = None
        image_binary = None


def calculateGoalPointToFrame(size_x, size_y, pointsFrame, dist, l, height, width):
    '''
    Функция от В.В.
    '''
    # Ищем ближайшую точку рамки к дрону
    d_min = min(dist)
    idx_min = list(dist).index(d_min)

    # Считаем коэффициент пересчета числа пикселей на метр в кадре
    k = l / math.sqrt(math.pow(pointsFrame.x[0]-pointsFrame.x[1], 2) + math.pow(pointsFrame.y[0]-pointsFrame.y[1], 2))

    # Считаем координаты точек рамки относительно дрона
    x = [0 for x in range(0, len(dist))]
    y = [0 for y in range(0, len(dist))]
    z = [0 for z in range(0, len(dist))]

    for i in range(0, len(dist)):
        d_norm = math.sqrt(math.pow((pointsFrame.x[i] - size_x / 2) * k, 2) + math.pow((pointsFrame.y[i] - size_y / 2) * k, 2) + math.pow(d_min, 2))

        x[i] = d_min + (dist[i] / d_norm - 1)
        y[i] = (size_x / 2 - pointsFrame.x[i]) * k * dist[i] / d_norm
        z[i] = (pointsFrame.y[i] - size_y / 2) * k * dist[i] / d_norm

    #print('x : ' + str(x))
    #print('y : ' + str(y))
    #print('z : ' + str(z))

    pointsDrone_ = pointsDrone(x, y, z)

    # Находим вектор нормали к плоскости рамки
    ax = pointsDrone_.x[1] - pointsDrone_.x[0]
    ay = pointsDrone_.y[1] - pointsDrone_.y[0]
    az = pointsDrone_.z[1] - pointsDrone_.z[0]
    bx = pointsDrone_.x[2] - pointsDrone_.x[0]
    by = pointsDrone_.y[2] - pointsDrone_.y[0]
    bz = pointsDrone_.z[2] - pointsDrone_.z[0]
    nx = ay * bz - by * az
    ny = bx * az - bz * ax
    nz = ax * by - ay * bx

    #print('nx : ' + str(nx))
    #print('ny : ' + str(ny))
    #print('nz : ' + str(nz))

    # Находим координаты центра рамки
    x_c = math.fsum(pointsDrone_.x) / 4
    y_c = math.fsum(pointsDrone_.y) / 4
    z_c = math.fsum(pointsDrone_.z) / 4

    #print('x_c : ' + str(x_c))
    #print('y_c : ' + str(y_c))
    #print('z_c : ' + str(z_c))

    # Находим точку p, удаленную от центра рамки на величину 1,25*width к дрону по направлению нормали
    n_norm = math.sqrt(math.pow(nx, 2) + math.pow(ny, 2) + math.pow(nz, 2))
    px1 = x_c - nx * 1.25 * width / n_norm
    py1 = y_c - ny * 1.25 * width / n_norm
    pz1 = z_c - nz * 1.25 * width / n_norm

    px2 = x_c + nx * 1.25 * width / n_norm
    py2 = y_c + ny * 1.25 * width / n_norm
    pz2 = z_c + nz * 1.25 * width / n_norm

    d1 = math.sqrt(math.pow(px1, 2) + math.pow(py1, 2) + math.pow(pz1, 2))
    d2 = math.sqrt(math.pow(px2, 2) + math.pow(py2, 2) + math.pow(pz2, 2))

    if d1 < d2:
        px = px1
        py = py1
        pz = pz1

    else:
        px = px2
        py = py2
        pz = pz2

    # print('px : ' + str(px))
    # print('py : ' + str(py))
    # print('pz : ' + str(pz))

    # Ищем угол между вектором нормали и вектором от дрона к точке p
    mx = x_c - px
    my = y_c - py
    mz = z_c - pz
    qx = -px
    qy = -py
    qz = -pz

    ang = math.acos((mx * qx + my * qy + mz * qz) / (math.sqrt(math.pow(mx, 2) + math.pow(my, 2) + math.pow(mz, 2)) * math.sqrt(math.pow(qx, 2) + math.pow(qy, 2) + math.pow(qz, 2))))
    #print('ang : ' + str(ang))

    # Если ang больше 165 градусов, то летим к центру рамки, в противном случае летим в точку p
    goal_ = goal(0,0,0)
    if ang > 170 * math.pi / 180:
        goal_.x = x_c
        goal_.y = y_c
        goal_.z = z_c
    else:
        goal_.x = px
        goal_.y = py
        goal_.z = pz

    return  goal_


# функция детектирования углов рамки
def frame_corners_detector():
    global detect_frame_publisher, window_detect_flag, cv_image, marker_publisher
    """
    return cords of corners
    """
    try:
        zeroes_image = np.zeros_like(image_binary)

        rgb_image_copy = rgb_image.copy()

        if rgb_image_copy is not None and 255 in image_binary:

            rgb_integrate = cv.bitwise_and(rgb_image_copy, rgb_image_copy, mask=image_binary)
            # cv.imshow('rgb_intagrate', rgb_integrate)
            hsv_image = cv.cvtColor(rgb_integrate, cv.COLOR_BGR2HSV)

            # делаем бинаризацию картинки
            image_mask = cv.inRange(hsv_image, (46, 127, 0), (255, 255, 255))

            image_mask = cv.erode(image_mask, None, iterations=5)

            # находим контуры
            contours, hierarchy = cv.findContours(image_mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

            # сортируем контуры
            contours = sorted(contours, key=cv.contourArea, reverse=True)

            cv.drawContours(zeroes_image, contours[0], -1, 255, 3)

            # Уменьшаем контуры белых объектов - делаем 2 итераций
            zeroes_image = cv.erode(zeroes_image, None, iterations=1)

            # cv.imshow("zeroes_image", zeroes_image)

            # Show Features to Track
            gray = zeroes_image.copy()

            # ищем хорошие точки для трекинга в углах рамки
            corners = cv.goodFeaturesToTrack(gray, 4, 0.4, 10)             #corners = cv.goodFeaturesToTrack(gray, 4, 0.01, 10)
            corners = np.int0(corners)
            corners = corners.reshape(4, -1)

            # print corners
            ###########################################
            # визуализация результатов работы функции #
            ###########################################
            # for i in corners:
            #     x, y = i.ravel()
            #     cv.circle(gray, (x, y), 3, 255, -1)
            #
            # cv.imshow('Gray', gray)

            # рисуем маркеры в найденых точках
            for i in corners:
                cv.drawMarker(rgb_image_copy, tuple(i), (0, 255, 0), markerType=cv.MARKER_TILTED_CROSS, thickness=2,
                              markerSize=50)

            cv.imshow("test", rgb_image_copy)

            ###########################################

            if cv.contourArea(contours[0]) > 20000. and corners is not None:
                print "Detect frame"
                window_detect_flag = True
                frame_detect_flag.detect_frame = True
                detect_frame_publisher.publish(frame_detect_flag)

                # time.sleep(2)

                size_x = rgb_image.shape[1]  # Размер кадра по х
                size_y = rgb_image.shape[0]  # Размер кадра по у

                pointsFrame.x = [corners[0][1],
                                 corners[1][1],
                                 corners[2][1],
                                 corners[3][1]]  # Координаты рамки в пикселях x       [1]

                pointsFrame.y = [corners[0][0],
                                 corners[1][0],
                                 corners[2][0],
                                 corners[3][0]]  # Координаты рамки в пикселях y       [0]

                # dist = [depth_frame_copy[corners[0][0]][corners[0][1]],
                #         depth_frame_copy[corners[1][0]][corners[1][1]],
                #         depth_frame_copy[corners[2][0]][corners[2][1]],
                #         depth_frame_copy[corners[3][0]][corners[3][1]]]  # Дистанции до углов рамки от дрона

                dist = np.array([depth_frame[corners[0][1]][corners[0][0]],
                        depth_frame[corners[1][1]][corners[1][0]],
                        depth_frame[corners[2][1]][corners[2][0]],
                        depth_frame[corners[3][1]][corners[3][0]]])  # Дистанции до углов рамки от дрона
                # print dist

                # print dist.max()

                if not math.isnan(dist.max()):
                    print "DIST OK"
                    # print list(dist).insdex()
                    goal_ = calculateGoalPointToFrame(size_x, size_y, pointsFrame, dist, l, height_of_drone, width_of_drone)

                    # goal_.x = goal_.x + 1.

                    print('x : ' + str(goal_.x) + ', ' + 'y : ' + str(goal_.y) + ', ' + 'z : ' + str(goal_.z))

                    point = Point(x=goal_.x, y=goal_.y, z=goal_.z)
                    marker = make_marker(point)
                    marker_publisher.publish(marker)
                    print('pub marker')

                    go_to = transform_cords_3D(drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z, roll, pitch, yaw, goal_)

                    goal_pose.pose.point.x = go_to[0]
                    goal_pose.pose.point.y = go_to[1]
                    goal_pose.pose.point.z = go_to[2]
                    goal_pose.pose.course = yaw

                    while True:
                        if not abs(goal_pose.pose.point.x - drone_pose.pose.position.x) < 0.3 and not abs(goal_pose.pose.point.y - drone_pose.pose.position.y) < 0.3:
                            goal_pose_pub.publish(goal_pose)
                            print abs(goal_pose.pose.point.x - drone_pose.pose.position.x), abs(goal_pose.pose.point.y - drone_pose.pose.position.y)

                        elif abs(goal_pose.pose.point.x - drone_pose.pose.position.x) < 0.3 and abs(goal_pose.pose.point.y - drone_pose.pose.position.y) < 0.3:
                            print "__DONE__"
                            frame_detect_flag.detect_frame = False
                            detect_frame_publisher.publish(frame_detect_flag)
                            break
            else:
                window_detect_flag = False
                frame_detect_flag.detect_frame = False
                detect_frame_publisher.publish(frame_detect_flag)
        else:
            print "Image not read"
    except:
        return None
        print "frame_corners_detector --> ERROR"


# функция нахождения столба
def detector_of_pillar():
    global flag, rgb_image_copy, cv_image

    rgb_image_copy = rgb_image.copy()

    if rgb_image_copy is not None and 255 in image_binary:

        rgb_integrate = cv.bitwise_and(rgb_image, rgb_image, mask=image_binary)

        half_bin = image_binary[: image_binary.shape[0] // 2, :]
        half_depth = depth_frame[: depth_frame.shape[0] // 2, :]

        # находим контуры
        contours, hierarchy = cv.findContours(half_bin, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

        # сортируем контуры
        contours = sorted(contours, key=cv.contourArea, reverse=True)

        if len(contours):
            cv.drawContours(rgb_integrate, contours[0], -1, (0, 255, 0), 5)

            cords_of_rect = cv.boundingRect(contours[0])
            cords_of_center = ((cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2))

            cv.rectangle(rgb_integrate, (cords_of_rect[0], cords_of_rect[1]), (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]), (255, 0, 0), 1)
            cv.circle(rgb_integrate, ((cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2)), 10, (0, 255, 0), -10)

            Q = -(cords_of_center[0] - (image_binary.shape[1] // 2)) * (1.3962 / image_binary.shape[1])      # 1.3962 - угол обзора камеры в радианах

            # print cords_of_center, Q
            dist_to_pillar = half_depth[cords_of_center[1]][cords_of_center[0]]

            x = math.cos(Q) * (dist_to_pillar + 2.0)
            y = math.sin(Q) * (dist_to_pillar + 1.5)

            cv.imshow("pillar", rgb_integrate)
            # print cv.contourArea(contours[0]), dist_to_pillar

            if cv.contourArea(contours[0]) > 10000. and dist_to_pillar < 2. and not window_detect_flag:
                print "detect pillar"
                frame_detect_flag.detect_frame = True
                detect_frame_publisher.publish(frame_detect_flag)

                # if flag:
                goal_pose.pose.point.x, goal_pose.pose.point.y = transform_cord(yaw, (x, y, 0.0))   # + Q, (dist_to_pillar + 1.5, 0.0, 0.0)

                goal_pose.pose.point.z = drone_alt
                goal_pose.pose.course = yaw + Q

                while True:
                    if not abs(goal_pose.pose.point.x - drone_pose.pose.position.x) < 0.3 and not abs(goal_pose.pose.point.y - drone_pose.pose.position.y) < 0.3:
                        goal_pose_pub.publish(goal_pose)
                        print abs(goal_pose.pose.point.x - drone_pose.pose.position.x), abs(goal_pose.pose.point.y - drone_pose.pose.position.y)

                    elif abs(goal_pose.pose.point.x - drone_pose.pose.position.x) < 0.3 and abs(goal_pose.pose.point.y - drone_pose.pose.position.y) < 0.3:
                        print "__DONE__"
                        # flag = True
                        frame_detect_flag.detect_frame = False
                        detect_frame_publisher.publish(frame_detect_flag)
                        break
            else:
                pass
                # frame_detect_flag.detect_frame = False
                # detect_frame_publisher.publish(frame_detect_flag)


def make_marker(point):
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.id = 1
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z

    return marker


def main():
    global depth_frame, image_binary, rgb_image, detect_frame_publisher, rgb_integrate, goal_pose_pub, marker_publisher

    rospy.init_node("Frame_detector_node")

    hz = rospy.Rate(5)

    # init subscribers
    rospy.Subscriber(depth_image_topic, Image, depth_image_cb)
    rospy.Subscriber(image_topic, Image, rgb_image_cb)
    rospy.Subscriber(drone_pose_topic, PoseStamped, drone_pose_cb)
    rospy.Subscriber(alt_topic, Float32, drone_alt_cb)

    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size=10)
    detect_frame_publisher = rospy.Publisher(frame_detect_topic, frame_detect, queue_size=10)
    marker_publisher = rospy.Publisher('window_target_marker', Marker)


    while not rospy.is_shutdown():

        if depth_frame is not None and rgb_image is not None:

            frame_corners_detector()

            # if window_detect_flag:
                # print corners


                    # goal_.x = goal_.x + 3.
                    # if goal_:
                    #     go_to = transform_cords_3D(drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z, roll, pitch, yaw, goal_)
                    #
                    #     goal_pose.pose.point.x = go_to[0]
                    #     goal_pose.pose.point.y = go_to[1]
                    #     goal_pose.pose.point.z = go_to[2]
                    #     goal_pose.pose.course = yaw
                    #
                    #     while True:
                    #         # while True:
                    #         if not abs(goal_pose.pose.point.x - drone_pose.pose.position.x) < 0.3 and not abs(goal_pose.pose.point.y - drone_pose.pose.position.y) < 0.3:
                    #             goal_pose_pub.publish(goal_pose)
                    #             print abs(goal_pose.pose.point.x - drone_pose.pose.position.x), abs(goal_pose.pose.point.y - drone_pose.pose.position.y)
                    #
                    #         elif abs(goal_pose.pose.point.x - drone_pose.pose.position.x) < 0.3 and abs(goal_pose.pose.point.y - drone_pose.pose.position.y) < 0.3:
                    #             print "__DONE__"
                    #             frame_detect_flag.detect_frame = False
                    #             detect_frame_publisher.publish(frame_detect_flag)
                    #             break

                        # if window_detect_flag:
                        #     goal_pose_pub.publish(goal_pose)
                        #     # time.sleep(10)
                            # print "go_to"

                    # print('x : ' + str(goal_.x) + ', ' + 'y : ' + str(goal_.y) + ', ' + 'z : ' + str(goal_.z))

            # else:
            #     print "Corners not detect"

            # detector_of_pillar()
            # pass
            hz.sleep()
        else:
            print "Images not read"

        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break

# начало исполнения кода
if __name__ == "__main__":
    main()
