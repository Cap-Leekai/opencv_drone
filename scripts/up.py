#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import rospy
import tf
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, TwistStamped
from drone_msgs.msg import Goal


# инициализация топиков
drone_pose_topic = "/mavros/local_position/pose"                    # топик текущей позиции
drone_goal_pose = "/goal_pose"

# переменные
goal_pose = Goal()
drone_pose = PoseStamped()
goal_vel = TwistStamped()
drone_alt = Float32()
yaw = Float32()
width_of_line = 0.2
goal_vel.twist.linear.x = 1.0

detect_frame_flag = False

ros_img = None
quaternion = None

# функция считывания текущего положения дрона
def drone_pose_cb(data):
    global drone_pose, quaternion, yaw
    drone_pose = data

    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)


# главная функция
def main():
    global yaw, quaternion
    rospy.init_node('fly_by_line_node')

    # init subscribers
    rospy.Subscriber(drone_pose_topic, PoseStamped, drone_pose_cb)
    # init publishers
    goal_pose_pub = rospy.Publisher(drone_goal_pose, Goal, queue_size = 10)

    print "_BEGIN_"

    hz = rospy.Rate(5)

    # основной цикл
    while not rospy.is_shutdown():
        if quaternion is not None:
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        else:
            continue
        goal_pose.pose.point.x = 0.0
        goal_pose.pose.point.y = 0.0
        goal_pose.pose.point.z = 1.5
        goal_pose.pose.course = yaw


        while True:
            if abs(goal_pose.pose.point.x - drone_pose.pose.position.x) > 0.2 or abs(
                    goal_pose.pose.point.y - drone_pose.pose.position.y) > 0.2 or abs(
                    goal_pose.pose.point.z - drone_pose.pose.position.z) > 0.2:
                goal_pose_pub.publish(goal_pose)
            else:
                break

        time.sleep(5)

        goal_pose.pose.point.x = 0.0
        goal_pose.pose.point.y = 0.0
        goal_pose.pose.point.z = 0.0
        goal_pose.pose.course = yaw

        while True:
            if abs(goal_pose.pose.point.x - drone_pose.pose.position.x) > 0.2 or abs(
                    goal_pose.pose.point.y - drone_pose.pose.position.y) > 0.2 or abs(
                goal_pose.pose.point.z - drone_pose.pose.position.z) > 0.2:
                goal_pose_pub.publish(goal_pose)
            else:
                break

        break

        if cv.waitKey(1) == 27:
            break

if __name__ == "__main__":
    main()
