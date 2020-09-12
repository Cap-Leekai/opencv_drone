#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import numpy as np
import rospy
import tf
import math
import dynamic_reconfigure.client

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

frame_detect_flag = frame_detect()
goal_pose = Goal()
drone_pose = PoseStamped()
drone_alt = Float32()
flag = True
window_detect_flag = False
use_unstable = False

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


def callback(config):
    rospy.loginfo("Config set to {run}".format(**config))


# функция считывания текущей высоты
def drone_alt_cb(data):
    global drone_alt
    drone_alt = data.data


# функция преобразования локальных координат в глобальные координаты
def transform_cord(W, cords):

    X = (math.cos(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) + (math.sin(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.cos(W) - cords[1] * math.sin(W))
    Y = (math.sin(W) * (drone_pose.pose.position.x * math.cos(W) + drone_pose.pose.position.y * math.sin(W))) - (math.cos(W) * (drone_pose.pose.position.x * math.sin(W) - drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.sin(W) + cords[1] * math.cos(W))
    return X, Y


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
        image_binary[(depth_frame < 4.9) & (depth_frame > 2.)] = 255

        image_binary = np.uint8(image_binary)

    except:
        print "Error read depth image"
        depth_image = None
        image_binary = None


# функция нахождения столба
def detector_of_pillar():
    global flag, rgb_image_copy, cv_image


    try:
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
                cords_of_center = (
                (cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2))

                cv.rectangle(rgb_integrate, (cords_of_rect[0], cords_of_rect[1]),
                             (cords_of_rect[0] + cords_of_rect[2], cords_of_rect[1] + cords_of_rect[3]),
                             (255, 0, 0), 1)
                cv.circle(rgb_integrate,
                          ((cords_of_rect[0] + cords_of_rect[2] // 2), (cords_of_rect[1] + cords_of_rect[3] // 2)),
                          10, (0, 255, 0), -10)

                Q = -(cords_of_center[0] - (image_binary.shape[1] // 2)) * (1.3962 / image_binary.shape[1])  # 1.3962 - угол обзора камеры в радианах

                # print cords_of_center, Q
                dist_to_pillar = half_depth[cords_of_center[1]][cords_of_center[0]]

                x = math.cos(Q) * (dist_to_pillar + 2.0)
                y = math.sin(Q) * (dist_to_pillar + 1.5)

                point = Point(x=x, y=y, z=drone_alt)
                marker = make_marker(point)
                marker_publisher.publish(marker)
                print('pub marker')

                cv.imshow("pillar", rgb_integrate)
                # print cv.contourArea(contours[0]), dist_to_pillar

                if cv.contourArea(contours[0]) > 10000. and dist_to_pillar < 2.5 and not window_detect_flag:
                    print "detect pillar"
                    frame_detect_flag.detect_frame = True
                    detect_frame_publisher.publish(frame_detect_flag)

                    use_unstable = True
                    client.update_configuration({"run": use_unstable})

                    # if flag:
                    goal_pose.pose.point.x, goal_pose.pose.point.y = transform_cord(yaw, (x, y, 0.0))  # + Q, (dist_to_pillar + 1.5, 0.0, 0.0)
                    goal_pose.pose.point.z = drone_alt
                    goal_pose.pose.course = yaw + Q

                    while True:
                        if not abs(goal_pose.pose.point.x - drone_pose.pose.position.x) < 0.3 and not abs(
                                goal_pose.pose.point.y - drone_pose.pose.position.y) < 0.3:
                            goal_pose_pub.publish(goal_pose)
                            print abs(goal_pose.pose.point.x - drone_pose.pose.position.x), abs(
                                goal_pose.pose.point.y - drone_pose.pose.position.y)

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
    except:
        print "Pillar detector --> ERROR"

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
    global depth_frame, image_binary, rgb_image, detect_frame_publisher, rgb_integrate, goal_pose_pub, marker_publisher, client

    rospy.init_node("Pillar_detector_node")

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

    # init client dyhamic reconfigure
    client = dynamic_reconfigure.client.Client("unstable_planner_node", timeout=1, config_callback=callback)

    while not rospy.is_shutdown():

        if depth_frame is not None and rgb_image is not None:
            pass
            detector_of_pillar()

            hz.sleep()
        else:
            print "Images not read"

        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break

# начало исполнения кода
if __name__ == "__main__":
    main()
