#coding=utf8
import numpy as np
import cv2
from matplotlib import pyplot as plt

def kaze_match():

    # load the image and convert it to grayscale
    global img1, img2
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # initialize the AKAZE descriptor, then detect keypoints and extract
    # local invariant descriptors from the image
    detector = cv2.AKAZE_create()
    (kps1, descs1) = detector.detectAndCompute(gray1, None)
    (kps2, descs2) = detector.detectAndCompute(gray2, None)

    print("keypoints: {}, descriptors: {}".format(len(kps1), descs1.shape))
    print("keypoints: {}, descriptors: {}".format(len(kps2), descs2.shape))

    # # Match the features
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(descs1, descs2, k = 2)    # typo fixed
    print(len(matches[0]))

    # Apply ratio test
    good = []
    for m,n in matches:
        if m.distance < 0.9*n.distance:
            good.append([m])

    iframe = cv2.drawMatchesKnn(img1, kps1, img2, kps2, good, None,
                          matchColor = (0, 255, 0), matchesMask = None,
                          singlePointColor = (255, 0, 0), flags = 2 )

    cv2.imshow('AKAZE matching', iframe)
def orb_match():
    global img1, img2

    orb = cv2.ORB_create()

    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)

    matches = bf.match(des1, des2)
    matches = sorted(matches, key = lambda x:x.distance)

    for i in kp1:
        print(len(matches))
        print(i)

    img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:100], None, flags = 2)
    plt.imshow(img3)
    plt.show()


if __name__ == "__main__":
    img1 = cv2.imread('logotip.png', 1)
    img2 = cv2.imread('logotip2.png', 1)       #scene.jpg

    while True:
        orb_match()
        if cv2.waitKey(1) == 27:
            break