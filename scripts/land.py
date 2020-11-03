#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import rospy
import tf
import time

from mavros_msgs.srv import CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from drone_msgs.msg import Goal
from opencv_drone.msg import frame_detect

# инициализация топиков
cam_img_topic = "/iris_rplidar/usb_cam/image_raw"                    # топик нижней камеры
drone_pose_topic = "/mavros/local_position/pose"                    # топик текущей позиции
drone_goal_pose = "/goal_pose"
frame_detect_topic = "/frame_detector"


# переменные
goal_pose = Goal()
drone_pose = PoseStamped()
goal_vel = TwistStamped()
frame_detect_flag = frame_detect()

quaternion = None

# функция считывания текущего положения дрона
def drone_pose_cb(data):
    global drone_pose, quaternion
    drone_pose = data
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)

def mav_land():
    landing = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
    landing(0.0, 0.0, 0.0, 0.0, 0.0)


# главная функция
def main():
    rospy.init_node('land_detector')

    # init subscribers
    rospy.Subscriber(drone_pose_topic, PoseStamped, drone_pose_cb)

    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size = 10)
    detect_frame_publisher = rospy.Publisher(frame_detect_topic, frame_detect, queue_size=10)

    print "_BEGIN_"

    hz = rospy.Rate(5)

    time.sleep(20)
    print "QQ"
    # основной цикл
    while not rospy.is_shutdown():
        if quaternion is not None:
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        else:
            continue

        if abs(drone_pose.pose.position.x) < 1.0 and abs(drone_pose.pose.position.y) < 1.0:
            # rospy.loginfo("X :%s Y :%s" %(drone_pose.pose.position.x, drone_pose.pose.position.y))
            rospy.loginfo("land")
            frame_detect_flag.detect_frame = True
            detect_frame_publisher.publish(frame_detect_flag)

            goal_pose.pose.point.x = 0.0
            goal_pose.pose.point.y = 0.0
            goal_pose.pose.point.z = 1.5
            goal_pose.pose.course = yaw

            while True:
                # print(abs(goal_pose.pose.point.x - drone_pose.pose.position.x), abs(goal_pose.pose.point.y - drone_pose.pose.position.y), abs(goal_pose.pose.point.z - drone_pose.pose.position.z))
                if abs(goal_pose.pose.point.x - drone_pose.pose.position.x) > 0.1 or abs(goal_pose.pose.point.y - drone_pose.pose.position.y) > 0.1 or abs(goal_pose.pose.point.z - drone_pose.pose.position.z) > 0.1:
                    goal_pose_pub.publish(goal_pose)
                else:
                    break

            mav_land()

        else:
           pass


if __name__ == "__main__":
    main()