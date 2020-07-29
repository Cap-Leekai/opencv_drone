#!/usr/bin/env python
#coding=utf8

import rospy
import cv2 as cv


from cv_bridge import CvBridge
from sensor_msgs.msg import Image


ros_image_forward = Image()


def img_cb(data):
    global ros_image_forward
    ros_image_forward = data


def main():
    global cv_image

    rospy.init_node('camera_frame_test')
    bridge = CvBridge()
    rospy.Subscriber('/mono_cam_forward/camera_mono/image_raw', Image, img_cb)

    while not rospy.is_shutdown():
        try:
            cv_image_forward = bridge.imgmsg_to_cv2(ros_image_forward, "bgr8")
            cv.imshow("hren", cv_image_forward)

        except:
            print("FAIL")

        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break


if __name__ == "__main__":
    main()
    cv.destroyAllWindows()
