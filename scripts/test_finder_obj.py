import numpy as np
import cv2
from matplotlib import pyplot as plt
#
# def kaze_match(im1_path, im2_path):
#
#     # load the image and convert it to grayscale
#     im1 = cv2.imread(im1_path)
#     im2 = cv2.imread(im2_path)
#     gray1 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
#     gray2 = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
#
#     # initialize the AKAZE descriptor, then detect keypoints and extract
#     # local invariant descriptors from the image
#     detector = cv2.AKAZE_create()
#     (kps1, descs1) = detector.detectAndCompute(gray1, None)
#     (kps2, descs2) = detector.detectAndCompute(gray2, None)
#
#     print("keypoints: {}, descriptors: {}".format(len(kps1), descs1.shape))
#     print("keypoints: {}, descriptors: {}".format(len(kps2), descs2.shape))
#
#     # # Match the features
#     bf = cv2.BFMatcher(cv2.NORM_HAMMING)
#     matches = bf.knnMatch(descs1,descs2, k = 2)    # typo fixed
#     print(kps1)
#
#     # # Apply ratio test
#     # good = []
#     # for m,n in matches:
#     #     if m.distance < 0.9*n.distance:
#     #         good.append([m])
#
#     # cv2.drawMatchesKnn expects list of lists as matches.
#     # iframe = cv2.drawMatchesKnn(im1, kps1, im2, kps2, good, None,
#     #                       matchColor = (0, 255, 0), matchesMask = None,
#     #                       singlePointColor = (255, 0, 0), flags = 2 )
#
#     # cv2.imshow('AKAZE matching', im1)



if __name__ == "__main__":
    while True:

        img1 = cv2.imread('scene.jpg')  # queryImage
        img2 = cv2.imread('logotype.jpg')  # trainImage

        cv2.imshow("1", img1)
        cv2.imshow("2", img2)
        # # Initiate SIFT detector
        # orb = cv2.ORB()

        # find the keypoints and descriptors with SIFT
        # kp1, des1 = orb.detectAndCompute(img1, None)
        # kp2, des2 = orb.detectAndCompute(img2, None)

        # create BFMatcher object
        # bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Match descriptors.
        # matches = bf.match(des1, des2)

        # Sort them in the order of their distance.
        # matches = sorted(matches, key=lambda x: x.distance)

        # Draw first 10 matches.
        # img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], flags=2)

        # plt.imshow(img3), plt.show()

        if cv2.waitKey(1) == 27:
            break