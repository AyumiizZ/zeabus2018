'''
    File name: test.py
    Author: zeabus2018
    Date created: 05/25/2018
    Python Version: 3.6.1
'''
import cv2 as cv
import csv
import numpy as np
from get_features_training import *

abs_path = 'C:/Users/skconan/Desktop/Workspace/dice/'
img_path = abs_path + 'images/'


def get_circle(img_bgr):
    result = []
    img_result = img_bgr.copy()
    area_ratio_expected = 0.8
    gray = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)
    _, mask = cv.threshold(gray, 90, 255, cv.THRESH_BINARY_INV)
    _, contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    for cnt in contours:
        (x, y), radius = cv.minEnclosingCircle(cnt)
        area_real = math.pi * radius * radius
        area_cnt = cv.contourArea(cnt)
        if area_cnt <= 20:
            continue
        area_ratio = area_cnt / area_real
        if area_ratio < area_ratio_expected:
            continue
        cv.drawContours(img_result, cnt, -1, (0, 255, 0), 1)
        result.append([int(x), int(y), int(radius), int(x**2) + int(y**2)])

    result = sorted(result, key=itemgetter(2))
    result_last = []
    for res in result:
        result_last.append(res[:-1])
    cv.imshow('image_result', img_result)
    cv.waitKey(-1)

    return result_last


def main():
    coeff = [0.00000000e+00, 3.58437238e-16, 1.02762883e-03, 1.45365559e-02, 2.61521935e-03, 1.01998667e-02,
             2.30336737e-03, 7.85740575e-03, 1.81348852e-02, -3.70559176e-01, -1.09143644e-02, -7.04152544e-04]
    img = cv.imread(img_path + 'dice_6_1.jpg', 1)
    img = cv.resize(img, (0, 0), fx=0.5, fy=0.5)
    circles = get_circle(img)
    n = len(circles)
    for i in [5,6, 2]:
        if n < i:
            continue
        index = get_index(n, i)
        x_variable = [0] * 12
        dice = i
        point = None
        min = 10
        ct_y = 0
        for j in index:
            ct = 0
            x_before = 0
            y_before = 0
            # print(j)
            for k in j:
                x, y, r = circles[k - 1]
                if ct == 0:
                    x_before = x
                    y_before = y
                x_variable[ct] = x - x_before
                ct += 1
                x_variable[ct] = y - y_before
                ct += 1
                x_before = x
                y_before = y
            y = 0
            for k in range(12):
                y += coeff[k] * x_variable[k]
            # print(y)
            if y > 0 and abs(y - dice) < min:
            # if True:
                img_result = img.copy()
                print(y, dice, min)
                min = abs(y - dice)
                point = j
            ct_y += 1
        for p in point:
            x, y, r = circles[p - 1]
            cv.circle(img_result, (x, y), r, (0, 0, 255), -1)
        cv.imshow('img_result', img_result)
        cv.waitKey(-1)


if __name__ == '__main__':
    main()
