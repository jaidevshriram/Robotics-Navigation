import warnings

import numpy as np
import cv2
import matplotlib.pyplot as plt

"""
Input: Binary image with obstacles in white, on a black background
Output: Array of tuples of the form (x, y, r) where x, y = position of blob, r = radius of blob
"""
def detect_obstacles(map):

    params = cv2.SimpleBlobDetector_Params()
    params.blobColor = 255
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False
    params.filterByArea = True
    params.maxArea=20000

    detector = cv2.SimpleBlobDetector_create(params)
    detector.empty()
    keypoints = detector.detect(map)

    obstacle_list = []

    for keypoint in keypoints:
        obstacle_list.append((*keypoint.pt, keypoint.size))

    # print(obstacle_list)

    # img_blob = cv2.drawKeypoints(map, keypoints, np.array([]), 255, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # plt.imshow(img_blob)
    # plt.show()

    return obstacle_list