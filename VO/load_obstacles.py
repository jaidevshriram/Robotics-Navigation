import warnings

import numpy as np
import matplotlib.pyplot as plt
import cv2
import matplotlib.pyplot as plt

"""
Input: Binary image with obstacles in white, on a black background
Output: Array of tuples of the form (x, y, r) where x, y = position of blob, r = radius of blob
"""
def detect_obstacles(map):

    params = cv2.SimpleBlobDetector_Params()
    params.blobColor = 255
    params.filterByCircularity = True
    params.filterByConvexity = False
    params.filterByInertia = False
    params.filterByArea = False

    detector = cv2.SimpleBlobDetector_create(params)
    detector.empty()
    keypoints = detector.detect(map)

    # img_blob = cv2.drawKeypoints(map, keypoints, np.array([]), 255, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # plt.imshow(img_blob)
    # plt.show()

    obstacle_list = []
    for keypoint in keypoints:
        obstacle_list.append([*keypoint.pt, keypoint.size/2])
    #     img_blob = cv2.drawKeypoints(map, keypoints, np.array([]), 255, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # plt.imshow(img_blob)
    # plt.show()

    # obstacle_list = []
    # one = [10, 10, 3]
    # obstacle_list.append(one)
    # two = [5, 3, 1]
    # obstacle_list.append(two)

    # two = [14, 13, 3]
    # obstacle_list.append(two)
    # two = [17, 3, 3]
    # obstacle_list.append(two)

    return obstacle_list
