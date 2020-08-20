#coding=utf8
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import time


MIN_MATCH_COUNT = 10                                # минимальное количество совпадений
img1 = cv.imread('22.jpg',0)          # queryImage
# img1 = cv.Canny(img1, 100, 200)

cap = cv.VideoCapture('/dev/video0')

# cap.set(cv.CAP_PROP_FPS, 24) # Частота кадров
# cap.set(cv.CAP_PROP_FRAME_WIDTH, 640) # Ширина кадров в видеопотоке.
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, 360) # Высота кадров в видеопотоке.

# Initiate SIFT detectorqq
sift = cv.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(img1, None)                           # получаем кей-поинты и дескрипторы из картинки

while True:

    ret, frame = cap.read()
    img2 = frame.copy()
    # print(len(img2), len(img2[0]))

    # img2 = cv.Canny(frame, 100, 200)
    # cv.imshow("Canny", frame)

    # find the keypoints and descriptors with SIFT
    kp2, des2 = sift.detectAndCompute(img2, None)                        # получаем кей-поинты и дескрипторы из картинки камеры
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

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)  # список с координатами точкек запроса
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)  # список с координатами точек обучения

        print(src_pts)
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape

        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

        try:
            dst = cv.perspectiveTransform(pts, M)
            img2 = cv.polylines(img2,[np.int32(dst)],True,255,3, cv.LINE_AA)

        except:
            continue

        print( "Matches found ! - {}/{}".format(len(good), MIN_MATCH_COUNT) )

    else:
        print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None

    draw_params = dict(matchColor = (0, 255, 0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)

    img3 = cv.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)

    # plt.imshow(img3, 'gray'),plt.show()

    cv.imshow('frame', img3)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
