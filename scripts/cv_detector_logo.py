#!/usr/bin/env python
#coding=utf8

import rospy
import cv2 as cv
import numpy as np
import math
import tf


from cv_bridge import CvBridge
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Quaternion
from drone_msgs.msg import Goal
from sensor_msgs.msg import Image

# класс хранящий основные параметры найденных контуров
class contour_obj:
    # конструктор
    def __init__(self):
        self.name = None
        self.cords = []
        self.mask = []

# флаги
view_window_flag = False    # фдаг отображения окон с результатами обработки изображений сделано для отладки

# переменные
drone_alt = 0.0             # текущая высота дрона
drone_pose = PoseStamped()  # текущая позиция дрона в глобальной системе координат
goal_point = Goal()         # целевая точка, в которую должен лететь дрон
max_resize = (64, 64)       # задаем максимальный размер кадра для "ресайза" выделенных контуров

# названия путей
logo_of_object = 'land_point_blue.png'
camera_file_port = "/dev/video2"                 # stereo elp >> /dev/video2, /dev/video4

# topics
alt_topic = "/drone/alt"                            # топик текущей высоты
drone_pose_topic = "/mavros/local_position/pose"    # топик текущей позиции
drone_goal_pose = "/goal_pose"                      # топик целевой точки
camera_server_topic = "/camera_server"              # топик передачи картинки на сервер просмотра(для удаленного отображения картинки на ПК управления)

# делаем захват видео с камеры в переменную cap
cap = cv.VideoCapture(camera_file_port)
cap.set(cv.CAP_PROP_FPS, 24) # Частота кадров
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280) # Ширина кадров в видеопотоке.
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720) # Высота кадров в видеопотоке.


# функция считывания текущего положения дрона
def call_back_Drone_Pose(data):
    global drone_pose, quaternion
    drone_pose = data
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)


# функция считывания текущей высоты
def call_back_Drone_Alt(data):
    global drone_alt
    drone_alt = data.data


# функция определяющая какой маркер обнаружен
def detect_marker(cut_frame, origin_frame_bin):
    difference_val = 0
    similarity_val = 0
    try:
        for i in range(64):
            for j in range(64):
                if cut_frame[i][j] == origin_frame_bin[i][j]:
                    similarity_val += 1
                elif cut_frame[i][j] != origin_frame_bin[i][j]:
                    difference_val += 1
    except:
        similarity_val = 0
        difference_val = 0

    return similarity_val, difference_val


# функция вырезает детектируемый контур из кадра и возвращает его в бинаризованном виде с фиксированным размером кадра
def cut_contour(frame, cords, minVal, maxVal):
    try:
        # print(cords)
        cut_contour_frame = frame[cords[1]: (cords[1] + cords[3]) + 1, cords[0]: (cords[0] + cords[2]) + 1]

        # делаем фиксированный размер картинки 64 x 64
        cut_contour_frame = cv.resize(cut_contour_frame, max_resize)

        hsv_local = cv.cvtColor(cut_contour_frame, cv.COLOR_BGR2HSV)
        cut_contour_frame = cv.inRange(hsv_local, minVal, maxVal)


    except:
        cut_contour_frame = None

    return cut_contour_frame


# функция выделения контуров
def contour_finder(frame, ValMinBGR, ValMaxBGR):
    # создаём объект хранящий в себе основные параметры детектируемого объекта
    detect_obj = contour_obj()

    # переводим картинку с камеры из формата BGR в HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # делаем размытие картинки HSV
    hsv = cv.blur(hsv, (4, 4))

    if view_window_flag:
        cv.imshow('Blur', hsv)

    # делаем бинаризацию картинки и пихаем её в переменную mask
    detect_obj.mask = cv.inRange(hsv, ValMinBGR, ValMaxBGR)              #OrangeMinBGR, OrangeMaxBGR
    # cv.imshow('mask', mask)

    # Уменьшаем контуры белых объектов - делаем две итерации
    detect_obj.mask = cv.erode(detect_obj.mask, None, iterations = 3)
    # cv.imshow("Erode", mask)

    # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
    detect_obj.mask = cv.dilate(detect_obj.mask, None, iterations = 3)

    if view_window_flag:
        cv.imshow('Dilate', detect_obj.mask)

    # ищем контуры в результирующем кадре
    contours = cv.findContours(detect_obj.mask, cv.RETR_TREE , cv.CHAIN_APPROX_NONE)         # cv.RETR_TREE

    # вычленяем массив контуров из переменной contours и переинициализируем переменную contours
    contours = contours[1]

    # проверяем найдены ли контуры в кадре
    if contours:
        # сортируем элементы массива контуров по площади по убыванию
        contours = sorted(contours, key = cv.contourArea, reverse = True)
        # выводим все контуры на изображении
        # cv.drawContours(frame, contours, -1, (0, 180, 255), 1)  # cv.drawContours(кадр, массив с контурами, индекс контура, цветовой диапазон контура, толщина контура)

        # получаем координаты прямоугольника описанного относительно контура
        detect_obj.cords = cv.boundingRect(contours[0])  # возвращает кортеж в формате  (x, y, w, h)
        return detect_obj

    else:
        return detect_obj

