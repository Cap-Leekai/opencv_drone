#!/usr/bin/env python
#coding=utf8

import rospy
import cv2 as cv


from cv_bridge import CvBridge
from sensor_msgs.msg import Image


camera_file_port = "/dev/video2"

ros_image_forward = Image()
ros_image_down = Image()


# cap_forward = cv.VideoCapture(camera_file_port)# stereo elp >> /dev/video2, /dev/video4
# cap_forward.set(cv.CAP_PROP_FPS, 30) # Частота кадров
# cap_forward.set(cv.CAP_PROP_FRAME_WIDTH, 640) # Ширина кадров в видеопотоке.
# cap_forward.set(cv.CAP_PROP_FRAME_HEIGHT, 360) # Высота кадров в видеопотоке.


def img_cb_d(data):
    global ros_image_down
    ros_image_down = data

def img_cb_f(data):
    global ros_image_forward
    ros_image_forward = data
    print("fordsdasdasd")

def main():
    # global cv_image

    rospy.init_node('camera_frame_test')
    bridge = CvBridge()
    bridge_sec = CvBridge()

    rospy.Subscriber('/mono_cam_down/camera_mono/image_raw', Image, img_cb_d)
    rospy.Subscriber('/mono_cam_forward/camera_mono_forward/image_raw', Image, img_cb_f)

    while not rospy.is_shutdown():
        try:
            cv_img_down = bridge.imgmsg_to_cv2(ros_image_down, "bgr8")
            # cv_img_forward = bridge_sec.imgmsg_to_cv2(ros_image_forward, "bgr8")
            # print("OK")
            cv.imshow("hren", cv_img_down)

        except:
            print("FAIL")

        if cv.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
            break


if __name__ == "__main__":
    main()
    cv.destroyAllWindows()
