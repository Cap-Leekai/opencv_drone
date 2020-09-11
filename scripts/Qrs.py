#!/usr/bin/env python
#coding=utf8

import cv2 as cv
import rospy
import numpy as np
import sys
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#cam topics
down_cam_topic = "/iris_rplidar/usb_cam/image_raw"
front_cam_topic = "/r200/image_raw"

#data from cams
def img_cb(data):
    global ros_img_down
    ros_img_down = data

def img_qw(data):
    global ros_img_front
    ros_img_front = data

def main():
    global ros_img_down
    global ros_img_front
    rospy.init_node("QrDetect_node")
    bridge = CvBridge()
    rospy.Subscriber(down_cam_topic, Image, img_cb)
    rospy.Subscriber(front_cam_topic, Image, img_qw)

    flag = 1 # camera change
    j = 0
    Qrs = np.zeros(8) # размер массива qr на полу

    while not rospy.is_shutdown():
        try:
            while True:
                if flag == 1:
                    cv_img_down = bridge.imgmsg_to_cv2(ros_img_down, "bgr8")
                    image_down = cv_img_down
                    qrCodeDetector_down = cv.QRCodeDetector()
                    decodedText, points, _ = qrCodeDetector_down.detectAndDecode(image_down)
                    cv.imshow("Image_Down", image_down)

                    # Qrs = np.array([decodedText])
                    if j == 0:
                        if decodedText is not None:
                            print ("down")
                            Qrs[j] = float(decodedText)
                            print(float(decodedText))
                            print(Qrs[j])
                            j = j + 1
                            flag = 2
                    else:
                        if decodedText is not None:
                            if Qrs[j-1] != float(decodedText):
                                print ("down")
                                Qrs[j] = float(decodedText)
                                print(float(decodedText))
                                print(Qrs[j])
                                j = j + 1
                                flag = 2
                            else:
                                print("old qr")

                elif flag == 2:

                    cv_img_front = bridge.imgmsg_to_cv2(ros_img_front, "bgr8")
                    image_front = cv_img_front
                    qrCodeDetector_front = cv.QRCodeDetector()
                    decodedText1, points1, _ = qrCodeDetector_front.detectAndDecode(image_front)
                    cv.imshow("Image_Front", image_front)
                    # Qrs = np.array([decodedText])
                    # print(decodedText)
                    if decodedText1 is not None:
                        print ("front")
                        print(float(decodedText1))
                        if float(decodedText1) == Qrs[j - 1]:
                            print("door")
                            flag = 1

                print(Qrs)

                if cv.waitKey(1) == 27: # проверяем была ли нажата кнопка esc
                    break
        except:
            pass


if __name__ == "__main__":
    main()
    cv.destroyAllWindows()