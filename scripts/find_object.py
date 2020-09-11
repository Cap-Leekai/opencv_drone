#coding=utf8

import rospy
import numpy as np
import cv2 as cv
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
ros_img = Image()

image_topic = "/iris_rplidar/usb_cam/image_raw"


# callback считывания картинки с realsence в rgb
def img_cb(data):
    global ros_img
    ros_img = data



def main():
    global cv_img
    rospy.init_node('cube_detector')

    rospy.Subscriber(image_topic, Image, img_cb)

    bridge = CvBridge()

    MIN_MATCH_COUNT = 10                                # минимальное количество совпадений
    img1 = cv.imread('logotype_on_blue.jpg', 0)          # queryImage

    # Initiate SIFT detector
    sift = cv.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)                           # получаем кей-поинты и дескрипторы из картинки

    while not rospy.is_shutdown():

        if ros_img:
            try:
                cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
                img2 = cv_img.copy()
                gray = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)

                # find the keypoints and descriptors with SIFT
                kp2, des2 = sift.detectAndCompute(gray, None)                        # получаем кей-поинты и дескрипторы из картинки камеры
                FLANN_INDEX_KDTREE = 1

                index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)       # создаем словарь с параметрами для FlannBasedMatcher

                search_params = dict(checks = 50)                                    # создаем словарь с параметрами для FlannBasedMatcher
                flann = cv.FlannBasedMatcher(index_params, search_params)
                matches = flann.knnMatch(des1, des2, k = 2)                          # исещм сходства между дескрипторами и записываем их в список matches

                # store all the good matches as per Lowe's ratio test.
                good = []

                for m, n in matches:
                    if m.distance < 0.45 * n.distance:
                        good.append(m)

                if len(good) > MIN_MATCH_COUNT:
                    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)  # список с координатами точкек запроса
                    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)  # список с координатами точек обучения

                    # print(src_pts)
                    M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
                    matchesMask = mask.ravel().tolist()
                    h, w = img1.shape

                    pts = np.float32([ [0, 0],[0, h-1],[w-1, h-1],[w-1, 0] ]).reshape(-1, 1, 2)

                    try:
                        dst = cv.perspectiveTransform(pts, M)
                        img2 = cv.polylines(img2, [np.int32(dst)], True, 255, 3, cv.LINE_AA)

                    except:
                        continue

                    print("Matches found ! - {}/{}".format(len(good), MIN_MATCH_COUNT))

                else:
                    print("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
                    matchesMask = None

                draw_params = dict(matchColor = (0, 255, 0), # draw matches in green color
                                   singlePointColor = None,
                                   matchesMask = matchesMask, # draw only inliers
                                   flags = 2)

                img3 = cv.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)

                # plt.imshow(img3, 'gray'), plt.show()

                cv.imshow('frame', img3)

                if cv.waitKey(1) == 27:
                    break
            except:
                print


if __name__ == "__main__":
    main()

