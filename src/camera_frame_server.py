#!/usr/bin/env python
#coding=utf8

import rospy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

camera_server_topic = "/camera_server"

def camera_frame_cb(data):
    global cv_image
    # print(data)
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    print("readed")

def main():

    while not rospy.is_shutdown():

        global bridge
        rospy.init_node('camera_frame_server')
        bridge = CvBridge()
        rospy.Subscriber(camera_server_topic, Image, camera_frame_cb)

        try:
            cv2.imshow("Frame_server1", cv_image)
            if cv2.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
                break
        except:
            print("Fail!")



if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()



