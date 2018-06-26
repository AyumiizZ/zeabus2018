#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

img_l = None
img_r = None


def normalization(np_array):
    max = np_array.max()
    min = np_array.min()
    norm = np.uint8(((np_array - min) / (max - min)) * 255)
    return norm


def nothing(var):
    return


def img_r_callback(msg):
    global img_r
    arr = np.fromstring(msg.data, np.uint8)
    img_r = cv.resize(cv.imdecode(
        arr, 1), (0, 0), fx=0.5, fy=0.5)
    img_r = cv.cvtColor(img_r, cv.COLOR_BGR2GRAY)


def img_l_callback(msg):
    global img_l
    arr = np.fromstring(msg.data, np.uint8)
    img_l = cv.resize(cv.imdecode(
        arr, 1), (0, 0), fx=0.5, fy=0.5)
    img_l = cv.cvtColor(img_l, cv.COLOR_BGR2GRAY)


def main():
    global img_l, img_r
    topic_l = '/stereo/left/image_rect_color/compressed'
    topic_r = '/stereo/right/image_rect_color/compressed'
    rospy.Subscriber(topic_l, CompressedImage, img_l_callback,  queue_size=10)
    rospy.Subscriber(topic_r, CompressedImage, img_r_callback,  queue_size=10)

    cv.namedWindow('image', flags=cv.WINDOW_NORMAL)
    num_disparities = 16
    block_size = 5
    cv.createTrackbar('NumDisparities', 'image', 1, 100, nothing)
    cv.createTrackbar('blockSize', 'image', 5, 255, nothing)
    cv.setTrackbarPos('NumDisparities', 'image', num_disparities)
    cv.setTrackbarPos('blockSize', 'image', block_size)
    while True:
        if img_l is None or img_r is None:
            continue
        if len(img_l.shape) > 2 or len(img_r.shape) > 2:
            continue

        num_disparities = cv.getTrackbarPos('NumDisparities', 'image')
        block_size = cv.getTrackbarPos('blockSize', 'image')

        if block_size % 2 == 0:
            block_size += 1
            if block_size < 5:
                block_size = 5
            cv.setTrackbarPos('blockSize', 'image', block_size)

        if not num_disparities % 16 == 0:
            if num_disparities < 16:
                num_disparities = 16
            elif num_disparities > 16:
                num_disparities = int(round(num_disparities / 16.0) * 16)
            cv.setTrackbarPos('NumDisparities', 'image', num_disparities)

        stereo = cv.StereoBM_create(
            numDisparities=num_disparities, blockSize=block_size)
        disparity = stereo.compute(img_l, img_r)
        norm = normalization(disparity)
        color = cv.applyColorMap(norm, cv.COLORMAP_JET)
        color = cv.resize(color, (0, 0), fx=0.5, fy=0.5)
        cv.imshow('image', color)
        k = cv.waitKey(1) & 0xff
        if k == ord('q'):
            break


if __name__ == '__main__':
    rospy.init_node('stereoBM')
    main()
