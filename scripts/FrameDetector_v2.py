#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import numpy as np
import rospy
from copy import deepcopy
import math
# import tf
# import pyrealsense2 as rs

from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point


depth_image_topic = "/camera/aligned_depth_to_infra1/image_raw"
image_topic = "/camera/color/image_raw"

view_result_flag = True
debug_prints = False

marker_publisher = None
contours = None
depth_frame = None
image_binary = None
rgb_image = None

old_time = 0.0
last_area = 0.0
l = 1.1                # Плечо рамки в метрах
height_of_drone = 0.6  # Высота дрона
width_of_drone = 0.8   # Ширина дрона
image_width_px = 1280
image_height_px = 720


# классы для функции пролета в рамку
class goal:
    def __init__(self, x0, y0, z0, x1, y1, z1):
        self.x0 = x0
        self.y0 = y0
        self.z0 = z0

        self.x1 = x1
        self.y1 = y1
        self.z1 = z1


class pointsFrame:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class pointsDrone:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def rgb_image_cb(data):
    global rgb_image
    try:
        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except Exception as e:
        print ("Error read rgb_image", e)
        rgb_image = None


# def valmap(value, istart, istop, ostart, ostop, clip_flag = True):
#     """
#     Re-maps a number from one range to another.
#     That is, a value of istart would get mapped to ostart,
#     a value of istop to ostop, values in-between to values in-between, etc.
#     :param value: value
#     :param istart:  the lower bound of the value’s current range
#     :param istop: the upper bound of the value’s current range
#     :param ostart: the lower bound of the value’s target range
#     :param ostop: the upper bound of the value’s target range
#     :return: The mapped value.
#     """
#     try:
#         val = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
#     except:
#         print("map error", value, istart, istop, ostart, ostop)
#         val = 0.0
#     if clip_flag:
#         return np.clip(val, ostart, ostop)
#     else:
#         return val
# берем конфигурацию основных переменных из сервера параметров ROS
def get_params_server():
    global depth_image_topic, view_result_flag, image_width_px, image_height_px

    depth_image_topic = rospy.get_param('~depth_image_topic', depth_image_topic)
    view_result_flag = rospy.get_param('~view_result_flag', view_result_flag)
    image_width_px = rospy.get_param('~image_width_px', image_width_px)
    image_height_px = rospy.get_param('~image_height_px', image_height_px)

    rospy.loginfo("init params done")


def depth_image_cb(data):
    global image_binary, depth_frame
    try:
        bridge = CvBridge()
        # переводим фрейм из росовского сообщения в картинку opencv
        depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
        depth_frame = np.array(depth_image, dtype=np.float32)   # каждый элемент фрейма хранит значение типа float являющееся расстоянием в метрах до точки

        image_binary = np.zeros_like(depth_frame)
        # делаем маску из допустимых пикселей на основе условия
        image_binary[(depth_frame < 3000) & (depth_frame > 100)] = 255
        # print 1
        image_binary = np.array(image_binary, dtype=np.uint8)
        # for i in range(len(image_binary)):
        #     print image_binary[i]

        # делаем размытие картинки image_binary
        image_binary = cv.blur(image_binary, (15, 15))

        # print("BEFORe",depth_frame[-1][-1])@
        # depth_frame = valmap(depth_frame, 0, 10000, 0, 255)
        # print("AFTER",depth_frame[-1][-1])

        # depth_frame = np.uint8(depth_frame)


    except:
        print ("Error read depth image")
        image_binary = None


def calculateGoalPointToFrame(size_x, size_y, pointsFrame, dist, l, height, width):
    '''
    Функция от В.В.
    '''
    # Ищем ближайшую точку рамки к дрону
    d_min = min(dist)
    idx_min = list(dist).index(d_min)

    # print "d_min : " + str(d_min)
    # print "idx_min : " + str(idx_min)

    x1_min = 1000
    x2_min = 1000
    idx1_min = -1
    idx2_min = -1

    for i in range(0, 2):
        if pointsFrame.x[i] < x1_min:
            x1_min = pointsFrame.x[i]
            idx1_min = i
        elif pointsFrame.x[i] < x2_min:
            x2_min = pointsFrame.x[i]
            idx2_min = i

    k = l / math.sqrt((pointsFrame.x[idx1_min] - pointsFrame.x[idx2_min]) ** 2 + (pointsFrame.y[idx1_min] - pointsFrame.y[idx2_min]) ** 2)

    # print "k : " + str(k)

    # Считаем координаты точек рамки относительно дрона
    x = [0 for x in range(0, len(dist))]                              # [0.0, 0.0, 0.0, 0.0]
    y = [0 for y in range(0, len(dist))]
    z = [0 for z in range(0, len(dist))]

    for i in range(0, len(dist)):
        # print "pointsFrame.x[i] : " + pointsFrame.x[i]
        # print "pointsFrame.y[i] : " + pointsFrame.y[i]
        # print "size_x : " + size_x
        # print "size_y : " + size_y

        d_norm = math.sqrt((((pointsFrame.x[i] - size_x / 2) * k ) ** 2) + (((pointsFrame.y[i] - size_y / 2) * k) ** 2) + d_min ** 2)

        # print "d_norm : " + str(d_norm)

        x[i] = d_min + (dist[i] / d_min - 1)
        y[i] = (size_x / 2 - pointsFrame.x[i]) * k * dist[i] / d_norm
        z[i] = (size_y / 2 - pointsFrame.y[i]) * k * dist[i] / d_norm

    # print('x : ' + str(x))
    # print('y : ' + str(y))
    # print('z : ' + str(z))

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

    # print('nx : ' + str(nx))
    # print('ny : ' + str(ny))
    # print('nz : ' + str(nz))

    # Находим координаты центра рамки
    x_c = math.fsum(pointsDrone_.x) / 4
    y_c = math.fsum(pointsDrone_.y) / 4
    z_c = math.fsum(pointsDrone_.z) / 4

    # print('x_c : ' + str(x_c))
    # print('y_c : ' + str(y_c))
    # print('z_c : ' + str(z_c))

    # Находим точку p, удаленную от центра рамки на величину 1,25*width к дрону по направлению нормали
    n_norm = math.sqrt(math.pow(nx, 2) + math.pow(ny, 2) + math.pow(nz, 2))
    px1 = x_c - nx * 1. * width / n_norm
    py1 = y_c - ny * 1. * width / n_norm
    pz1 = z_c - nz * 1. * width / n_norm

    px2 = x_c + nx * 1. * width / n_norm
    py2 = y_c + ny * 1. * width / n_norm
    pz2 = z_c + nz * 1. * width / n_norm

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
    goal_ = goal(0, 0, 0, 0, 0, 0)

    # записываем координаты центра рамки в объект целевой точки
    goal_.x1 = x_c
    goal_.y1 = y_c
    goal_.z1 = z_c

    if ang > 170 * math.pi / 180:
        goal_.x0 = x_c
        goal_.y0 = y_c
        goal_.z0 = z_c
    else:
        goal_.x0 = px
        goal_.y0 = py
        goal_.z0 = pz

    return goal_


def make_marker(point, id):
    marker = Marker()
    marker.header.frame_id = "/camera_link"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.id = id
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z

    return marker


def main():
    global old_time, last_area
    rospy.init_node("Frame_detector_node")
    # hz = rospy.Rate(50)

    get_params_server()

    # init subscribers
    rospy.Subscriber(depth_image_topic, Image, depth_image_cb)
    rospy.Subscriber(image_topic, Image, rgb_image_cb)

    marker_publisher = rospy.Publisher('window_target_marker', Marker)


    while not rospy.is_shutdown():
        # try:
        # pass
        if depth_frame is not None and image_binary is not None:
            try:

                edges = cv.Canny(image_binary, 150, 200)
                # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
                edges = cv.dilate(edges, None, iterations=1)
                if view_result_flag:
                    cv.imshow("test", edges)

            except:
                continue

            contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            if len(contours):
                zeroes_mask = np.zeros_like(image_binary)
                contours = sorted(contours, key=cv.contourArea, reverse=True)
                # rospy.loginfo(len(contours))

                list_cnt = []
                for i in contours:
                    if cv.contourArea(i) > 30000.0:
                        list_cnt.append(i)

                # rospy.loginfo(len(list_cnt))

                if len(list_cnt) == 0:
                    continue

                hull = cv.convexHull(list_cnt[0])

                epsilon = 0.05 * cv.arcLength(hull, True)
                approx = cv.approxPolyDP(hull, epsilon, True)

                ###
                # найдем производные от площади контура, чтобы понять есть ли резкий скочек площади, что будет означать, что контур детектируется недостаточно хорошо
                dt = rospy.get_time()-old_time
                old_time = rospy.get_time()
                # print ("asd", (cv.contourArea(approx) - last_area))
                Ft = ((cv.contourArea(approx) - last_area)/1000) / dt
                last_area = cv.contourArea(approx)
                ###
                rospy.loginfo("Ft: %s" %abs(Ft))

                # rospy.loginfo(cv.contourArea(approx))
                cv.drawContours(zeroes_mask, approx, -1, 255, 5)

                # cv.polylines(zeroes_mask, [approx], -1, 255, 4)
                # zeroes_mask = cv.dilate(zeroes_mask, None, iterations=)

                # ищем хорошие точки для трекинга в углах рамки
                corners = cv.goodFeaturesToTrack(zeroes_mask, 4, 0.4, 10)  # return [x:640, y:480]      #corners = cv.goodFeaturesToTrack(gray, 4, 0.01, 10)
                corners = np.int0(corners)

                if cv.contourArea(approx) > 30000. and corners is not None:
                    # print "Detect frame"
                    window_detect_flag = True
                    try:
                        corners = corners.reshape(4, -1)
                        # рисуем маркеры в найденых точках
                        for i in corners:
                            cv.drawMarker(zeroes_mask, tuple(i), 255, markerType=cv.MARKER_TILTED_CROSS, thickness=2,
                                          markerSize=50)
                        if view_result_flag:
                            cv.imshow("Contour", zeroes_mask)

                        size_x = zeroes_mask.shape[1]  # Размер кадра по х
                        size_y = zeroes_mask.shape[0]  # Размер кадра по у

                        pointsFrame.y = [corners[0][1],
                                         corners[1][1],
                                         corners[2][1],
                                         corners[3][1]]  # Координаты рамки в пикселях x       [1]

                        pointsFrame.x = [corners[0][0],
                                         corners[1][0],
                                         corners[2][0],
                                         corners[3][0]]  # Координаты рамки в пикселях y       [0]

                        dist = np.array([depth_frame[corners[0][1]][corners[0][0]],
                                         depth_frame[corners[1][1]][corners[1][0]],
                                         depth_frame[corners[2][1]][corners[2][0]],
                                         depth_frame[corners[3][1]][corners[3][0]]])
                        dist = dist / 1000


                        # проверяем есть ли нули в массиве дистанций, отсеиваем итерации с нулями
                        if not 0.0 in dist:
                            print "DIST OK"
                            # rospy.loginfo(dist)
                            goal_ = calculateGoalPointToFrame(size_x, size_y, pointsFrame, dist, l, height_of_drone, width_of_drone)

                            # print('x : ' + str(goal_.x0) + ', ' + 'y : ' + str(goal_.y0) + ', ' + 'z : ' + str(goal_.z0))

                            point0 = Point(x=goal_.x0, y=goal_.y0, z=goal_.z0)
                            point1 = Point(x=goal_.x1, y=goal_.y1, z=goal_.z1)
                            marker0 = make_marker(point0, 0)
                            marker1 = make_marker(point1, 1)

                            marker_publisher.publish(marker0)
                            marker_publisher.publish(marker1)
                            # print('pub marker')

                            #################################
                            # находим координату вектора от точки перед рамкой до точки в центре рамки относительно 0 точки системы координат
                            goal_vect = goal(goal_.x1 - goal_.x0, goal_.y1 - goal_.y0, goal_.z1 - goal_.z0, 0, 0, 0)

                            # находим угол смещения курса коптера @от нормали к плоскости рамки
                            yaw_error = math.acos(((goal_.x1 * goal_vect.x0) + (0 * goal_vect.y0))/(math.hypot(goal_vect.x0, goal_vect.y0) * math.hypot(goal_.x1, 0)))    # обычное скалярное произведение векторов
                            if goal_vect.y0 < 0:
                                yaw_error = -yaw_error

                            # rospy.loginfo("yaw_error: %s" %yaw_error)

                            #################################


                    except:
                        continue
                # rect = cv.minAreaRect(contours[1])
                # box = cv.boxPoints(rect)  # cv2.boxPoints(rect) for OpenCV 3.x
                # box = np.int0(box)

                # print len(contours)
                # cv.drawContours(zeroes_mask, [box], 0, 255, 2)

                # cv.drawContours(zeroes_mask, approx, -1, 255, 3)
                # cv.imshow('zeroes_mask', zeroes_mask)

            # except:
            #     continue
            #     rospy.loginfo_throttle(1, "Contours not find")


                # rospy.loginfo(len(contours))






                    # edges = cv.Canny(image_binary_blur, 100, 255)
                    # cv.imshow("edge",edges)


            # except Exception as e:
            #     print(e)

        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break

if __name__ == "__main__":
        main()