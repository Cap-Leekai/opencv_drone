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


# задаем пороги цвета
# диапазон синего круга в метке посадки загруженной из папки с проектом
BLUE_MIN_BGR = (104, 104, 0)        # orange(0, 135, 100)
BLUE_MAX_BGR = (255, 255, 255)       # orange(94, 255, 255)

# диапазон зеленого круга в метке посадки загруженной из папки с проектом
GREEN_MIN_BGR = (44, 69, 0)           # (45, 63, 0)
GREEN_MAX_BGR = (80, 146, 255)        # (80, 255, 162)

# диапазон синего круга в метке посадки детектируемой камерой
POINT_LAND_MIN_BLUE = (201, 0, 0)        # blue
POINT_LAND_MAX_BLUE = (255, 10, 255)

# диапазон зеленого круга в метке посадки детектируемой камерой
POINT_LAND_MIN_GREEN = (0, 146, 0)
POINT_LAND_MAX_GREEN = (161, 255, 255)

# флаги
view_window_flag = False    # фдаг отображения окон с результатами обработки изображений сделано для отладки
landing_flag = False        # флаг посадки

# переменные
drone_alt = 0.0             # текущая высота дрона
drone_pose = PoseStamped()  # текущая позиция дрона в глобальной системе координат
goal_point = Goal()         # целевая точка, в которую должен лететь дрон
max_resize = (64, 64)       # задаем максимальный размер кадра для "ресайза" выделенных контуров

# названия фреймов из проекта

point_of_land_img = 'land_point_blue.png'


# topics
alt_topic = "/drone/alt"                            # топик текущей высоты
drone_pose_topic = "/mavros/local_position/pose"    # топик текущей позиции
drone_goal_pose = "/goal_pose"                      # топик целевой точки
camera_server_topic = "/camera_server"              # топик передачи картинки на сервер просмотра(для удаленного отображения картинки на ПК управления)

# делаем захват видео с камеры в переменную cap
cap = cv.VideoCapture("/dev/video0")  # stereo elp >> /dev/video2, /dev/video4


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


# функция преобразования локальных координат в глобальные координаты
def transform_cord(W, cords):

    X = (math.cos(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) + (math.sin(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.cos(W) - cords[1] * math.sin(W))
    Y = (math.sin(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) - (math.cos(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.sin(W) + cords[1] * math.cos(W))
    print (W, X, Y)
    # glob_cords = np.dot(cords, matrix_transform)
    # X = glob_cords[0]
    # Y = glob_cords[1]
    return X, Y


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


# функция посадки
def land():
    
    while drone_alt > 0.30:
        h = drone_alt - 0.1
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        goal_point.pose.course = yaw
        goal_point.pose.point.x = drone_pose.pose.position.x
        goal_point.pose.point.y = drone_pose.pose.position.y
        goal_point.pose.point.z = h
        goal_pose_pub.publish(goal_point)

# основная функция
def main():
    
    rospy.init_node('cv_camera_capture') # инициальизируем данную ноду с именем cv_camera_capture
    bridge = CvBridge()

    # инициализируем подписку на топик
    rospy.Subscriber(drone_pose_topic, PoseStamped, call_back_Drone_Pose)
    rospy.Subscriber(alt_topic, Float32, call_back_Drone_Alt)

    global goal_pose_pub
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size = 10)
    camera_server_pub = rospy.Publisher(camera_server_topic, Image, queue_size = 1)

    hz = rospy.Rate(50)
    
    # инициализируем все переменные хранящие маски детектируемых картинок из памяти
    global point_land_mask_blue, point_land_mask_green

    # считываем и бинаризуем все метки детектирования
    point_land = cv.imread(point_of_land_img)

    point_land_mask_blue = cv.inRange(point_land, POINT_LAND_MIN_BLUE, POINT_LAND_MAX_BLUE)
    point_land_mask_blue = cv.resize(point_land_mask_blue, max_resize)
#    cv.imshow('cut_bin_blue', point_land_mask_blue)


    point_land_mask_green = cv.inRange(point_land, POINT_LAND_MIN_GREEN, POINT_LAND_MAX_GREEN)
    point_land_mask_green = cv.resize(point_land_mask_green, max_resize)
#    cv.imshow('cut_bin_green', point_land_mask_green)


    while not rospy.is_shutdown():


        # читаем флаг подключения камеры и картинку с камеры
        ret, frame = cap.read()
        # делаем копию кадра
        copy_frame = frame.copy()


        if ret:

            # cv.imshow("frame", frame)
            # получаем бъект контура по указанному интервалу цвета
            point_land_blue = contour_finder(frame, BLUE_MIN_BGR, BLUE_MAX_BGR)
            # print(point_land_blue.cords)
#            cv.imshow("point_blue", point_land_blue.mask)

            # получаем бъект контура по указанному интервалу цвета
            point_land_green = contour_finder(frame, GREEN_MIN_BGR, GREEN_MAX_BGR)
            # print(point_land_green.cords)
#            cv.imshow("point_green", point_land_green.mask)

            # сравниваем маски с камеры и маску сделанную из файлов
            marker_blue = detect_marker(cut_contour(copy_frame, point_land_blue.cords, BLUE_MIN_BGR, BLUE_MAX_BGR),
                                            point_land_mask_blue)
            marker_green = detect_marker(cut_contour(copy_frame, point_land_blue.cords, GREEN_MIN_BGR, GREEN_MAX_BGR),
                                            point_land_mask_green)


                # print("BLUE Найдено сходств %s, найдено различий %s" % marker_blue)
                # print("Green Найдено сходств %s, найдено различий %s" %  marker_green )


            # проверяем сходство детектырованных масок и масок картинок зашитых в файл с проектом
            if marker_blue[0] - marker_blue[1] > 2900 and marker_green[0] - marker_green[1] > 2900:
                print("marker of land True ")
                landing_flag = True

            else:
                print("marker of land False")
                landing_flag = False

            try:
                print("DEBUG = ", len(point_land_green))

                if point_land_green:
                    # рисуем прямоугольник описанный относительно контура
                    cv.rectangle(copy_frame,
                                 (point_land_green.cords[0], point_land_green.cords[1]),
                                 (point_land_green.cords[0] + point_land_green.cords[2],
                                  point_land_green.cords[1] + point_land_green.cords[3]),
                                 (0, 255, 0), 2)

                    # рисуем окружность в центре детектируемого прямоугольника
                    cv.circle(copy_frame,
                              (point_land_green.cords[0] + (point_land_green.cords[2] // 2),
                               point_land_green.cords[1] + (point_land_green.cords[3] // 2)), 5, (0, 255, 0), thickness=2)

                    cv.circle(frame,
                              (len(copy_frame[0]) // 2, len(copy_frame) // 2), 5, (0, 255, 0), thickness=2)

                    image_message = bridge.cv2_to_imgmsg(copy_frame, encoding="passthrough")
                    camera_server_pub.publish(image_message)
            except:
                print("Fail pub frame to server")

            # проверяем был ли обнаружен маркер посадки и если да, производим выполнение кода навигации
            if landing_flag:

                print("LANDING!")
                try:
                    # вычисляем локальные координаты метки в кадре камеры(измерение в пиксельных единицах!!!!)
                    X = (point_land_green.cords[0] + (point_land_green.cords[2] / 2)) - len(frame[0]) / 2
                    Y = - ((point_land_green.cords[1] + (point_land_green.cords[3] / 2)) - len(frame) / 2)

                     # считаем локальные координаты точки посадки в метрах(значения 21.8 и 16.1 это есть углы обзора камеры найденные экспериментальным путем)
                    glob_transform_cords = np.array([math.tan((21.8 / 320.0) * (math.pi / 180.0) * float(X)) * drone_alt, math.tan((16.1 / 240.0) * (math.pi / 180.0) * float(Y)) * drone_alt, 0.0])

                     # считаем углы поворота дрона из кватерниона в углы эйлера
                    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

                    glob_X, glob_Y = transform_cord(yaw, glob_transform_cords)  # пересчитываем найденные локальные координаты в глобальные
                    print ("X = %s, Y = %s, Z = %s" %(glob_X, glob_Y, drone_alt))
                    goal_point.pose.course = yaw
                    goal_point.pose.point.x = glob_X
                    goal_point.pose.point.y = glob_Y
                    goal_point.pose.point.z = drone_alt  #!#!#!#
                    goal_pose_pub.publish(goal_point)

                except:
                    print("Oops! Fail!")
                                 

            if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
                break

            hz.sleep()

        else:
            print("Camera not found!")
            break


if __name__ == "__main__":
    main()
    cap.release()
    cv.destroyAllWindows()

