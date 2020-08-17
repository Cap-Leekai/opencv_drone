#coding=utf8
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt

def detector_vicinities():
    global train_pts

    vicinities = []

    for point in train_pts:  # берем элемент из списка наилучших точек на картинке "камеры"
        good_train_points = []

        for next_point in train_pts:                                                                                    # берем потенциально следующий элемент из списка наилучших токек на картинке "камеры"
            if point[0] - next_point[0] > 0.1 and point[1] - next_point[1] > 0.1:                                       # проверяем не взяли ли мы точно такой же элемент next_point как point
                # print "point -> ", point, "___", "next_point -> ", next_point
                if abs(point[0] - next_point[0]) < 25.0 and abs(point[1] - next_point[1] < 25.0):
                    good_train_points.append(next_point)
                    # print good_train_points
        # print "len", len(good_train_points), "/", len(train_pts)

        if len(good_train_points) > 5:
            print "vicinities finded"
            vicinities.append(good_train_points)

    # print "Current Len of vicinities -> ", len(vicinities),"___", vicinities
    if len(vicinities):
        # print len(vicinities)
        for i in range(len(vicinities)):
            print i
            print vicinities[i-1]

def orb_match():
    global img1, img2, img2_copy, train_pts

    orb = cv2.ORB_create()     #ORB_create

    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)
    # print(kp1[0].pt)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)

    try:
        matches = bf.match(des1, des2)
        matches = sorted(matches, key = lambda x:x.distance)

        # print(matches[0].trainIdx)

        img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:40], None, flags = 2)
        cv2.imshow("Result", img3)

        # Homography
        if len(matches) > 10:
            query_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            train_pts = np.float32([kp2[m.trainIdx].pt for m in matches])


            # print("Len: %s Query_points: %s" %(len(query_pts), query_pts))
            # print("Len: %s Train_points: %s" %(len(train_pts), train_pts))

            ###############################################################################
            # Группируем ближайшие друг к другу точки
            ###############################################################################

            detector_vicinities()

            ###############################################################################
            # matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
            # matches_mask = mask.ravel().tolist()
            #
            # # Perspective transform
            # h, w = img1.shape
            #
            # pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            # dst = cv2.perspectiveTransform(pts, matrix)
            #
            # homography = cv2.polylines(img2, [np.int32(dst)], True, (255, 0, 0), 5)
            #
            # cv2.imshow("homography", homography)
        else:
            pass

    except:
        print("FAIL")
# def kaze_match():
#
#     # load the image and convert it to grayscale
#     global img1, img2
#
#     # initialize the AKAZE descriptor, then detect keypoints and extract
#     # local invariant descriptors from the image
#     detector = cv2.AKEZE_create()
#     (kps1, descs1) = detector.detectAndCompute(img1, None)
#     (kps2, descs2) = detector.detectAndCompute(img2, None)
#
#     print("keypoints: {}, descriptors: {}".format(len(kps1), descs1.shape))
#     print("keypoints: {}, descriptors: {}".format(len(kps2), descs2.shape))
#
#     # # Match the features
#     bf = cv2.BFMatcher(cv2.NORM_HAMMING)
#     matches = bf.knnMatch(descs1, descs2, k = 2)    # typo fixed
#
#
#     # Apply ratio test
#     good = []
#     for m,n in matches:
#         if m.distance < 0.9*n.distance:
#             good.append([m])
#
#     iframe = cv2.drawMatchesKnn(img1, kps1, img2, kps2, good[:15], None,
#                           matchColor = (0, 255, 0), matchesMask = None,
#                           singlePointColor = (255, 0, 0), flags = 2 )
#
#     cv2.imshow('AKAZE matching', iframe)
#
# def sift_and_flan():
#
#     global img1, img2
#
#     sift = cv2.SIFT_create()
#     kp1, desc1 = sift.detectAndCompute(img1, None)
#     kp2, desc2 = sift.detectAndCompute(img2, None)
#
#     index_params = dict(algorithm = 1, trees = 5)
#     search_params = dict(checks = 50)
#     flann = cv2.FlannBasedMatcher(index_params, search_params)
#     matches = flann.knnMatch(desc1, desc2, k = 2)
#     try:
#         good_points = []
#
#         for m, n in matches:
#             if m.distance < 0.7*n.distance:
#                 good_points.append(m)
#                 print("Вывод matches m = ", m.distance, " n = ", n.distance)
#
#         frame = cv2.drawMatches(img1, kp1, img2, kp2, good_points[:10], img2)
#         cv2.imshow("points", frame)
#
#         # Homography
#         if len(good_points) > 10:
#             query_pts = np.float32([kp1[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
#             train_pts = np.float32([kp2[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
#
#             print(query_pts, train_pts)
#             matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
#             matches_mask = mask.ravel().tolist()
#
#             # Perspective transform
#             h, w = img1.shape
#
#             pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
#             dst = cv2.perspectiveTransform(pts, matrix)
#
#
#             homography = cv2.polylines(img2, [np.int32(dst)], True, (255, 0, 0), 5)
#
#             cv2.imshow("homography", homography)
#         else:
#             cv2.imshow("img", img2)
#     except:
#         print("FAIL")
#
# def sift_create():
#
#     # Initiate SIFT detector
#     sift = cv2.ORB_create()
#
#     # find the keypoints and descriptors with SIFT
#     kp1, des1 = sift.detectAndCompute(img1, None)
#     kp2, des2 = sift.detectAndCompute(img2, None)
#     try:
#         # FLANN parameters
#         FLANN_INDEX_KDTREE = 0
#         index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
#         search_params = dict(checks=50)  # or pass empty dictionary
#
#         flann = cv2.FlannBasedMatcher(index_params, search_params)
#
#         matches = flann.knnMatch(des1, des2, k=2)
#
#         # Need to draw only good matches, so create a mask
#         matchesMask = [[0, 0] for i in xrange(len(matches))]
#
#         # ratio test as per Lowe's paper
#         for i, (m, n) in enumerate(matches):
#             if m.distance < 0.7 * n.distance:
#                 matchesMask[i] = [1, 0]
#
#         draw_params = dict(matchColor=(0, 255, 0),
#                            singlePointColor=(255, 0, 0),
#                            matchesMask=matchesMask,
#                            flags=0)
#
#         img3 = cv2.drawMatchesKnn(img1, kp1, img2, kp2, matches, None, **draw_params)
#
#         plt.imshow(img3)
#         plt.show()
#     except:
#         print("FAIL")
#
# def test_canny():
#
#     orb = cv2.ORB_create()
#
#     kp1, des1 = orb.detectAndCompute(img1, None)
#     kp2, des2 = orb.detectAndCompute(img2, None)
#
#     img3 = cv2.drawKeypoints(img1, kp1, img2)
#
#     cv2.imshow("Test", img3)
#
# def cut_contour(frame, cords):
#     try:
#         # print(cords)
#         cut_contour_frame = frame[cords[1]: (cords[1] + cords[3]) + 1, cords[0]: (cords[0] + cords[2]) + 1]
#
#         # делаем фиксированный размер картинки 64 x 64
#         cut_contour_frame = cv2.resize(cut_contour_frame, (64, 64))
#
#     except:
#         cut_contour_frame = None
#
#     return cut_contour_frame
#
# def orb_and_flann():
#     # Initiate SIFT detector
#     sift = cv2.ORB_create()
#
#     # find the keypoints and descriptors with SIFT
#     kp1, des1 = sift.detectAndCompute(img1, None)
#     kp2, des2 = sift.detectAndCompute(img2, None)
#
#     try:
#         # FLANN parameters
#         FLANN_INDEX_KDTREE = 1
#         FLANN_INDEX_LSH = 6
#         index_params = dict(algorithm=FLANN_INDEX_LSH,
#                             table_number=6,  # 12
#                             key_size=12,  # 20
#                             multi_probe_level=1)  # 2
#
#         search_params = dict(checks=50)  # or pass empty dictionary
#
#         flann = cv2.FlannBasedMatcher(index_params, search_params)
#
#         matches = flann.knnMatch(des1, des2, k=2)
#
#         # print(len(matches))
#
#         good_points = []
#         for m, n in matches:
#             if m.distance < 0.9 * n.distance:
#                 good_points.append(m)
#
#         # Need to draw only good matches, so create a mask
#         matchesMask = [[0, 0] for i in xrange(len(matches))]
#         #
#         # # ratio test as per Lowe's paper
#         # for i, (m, n) in enumerate(matches):
#         #     if m.distance < 0.7 * n.distance:
#         #         matchesMask[i] = [1, 0]
#         #
#         draw_params = dict(matchColor=(0, 255, 0),
#                            singlePointColor=(255, 0, 0),
#                            matchesMask=matchesMask,
#                            flags=0)
#         print(len(good_points))
#         img3 = cv2.drawMatchesKnn(img1, kp1, img2, kp2, matches, None, **draw_params)
#
#         cv2.imshow("test", img3)
#     except:
#         print("fail")
#     # plt.imshow(img3, ), plt.show()


if __name__ == "__main__":

    img1 = cv2.imread('blue.jpg', 0)
    cap = cv2.VideoCapture("passing_gazebo.mp4")


    while True:

        ret, frame = cap.read()
        if ret:
            try:
                # cv2.imshow('test', frame)
                # img2 = frame.copy()
                frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                frame_mask = cv2.inRange(frame_hsv, (0, 151, 75), (255, 255, 255))
                img2 = cv2.bitwise_and(frame, frame, mask = frame_mask)
                # cv2.imshow('test', img2)
                img2_copy = img2.copy()

                orb_match()
                print "__FINISH_OF_ITARATION__"
                time.sleep(0)
            except:
                print("Fatal fail!")
        else:
            print("Image2 not readed")

        if cv2.waitKey(1) == 27:
            break