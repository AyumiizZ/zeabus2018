#!/usr/bin/python2.7
'''
    File name: dice_detection.py
    Author: zeabus2018
    Date created: 05/25/2018
    Python Version: 3.6.1
'''
import cv2 as cv
import csv
import numpy as np
from lib import *
# from ../vision_lib import *
from operator import itemgetter

pattern = []
pattern_predict = []
width = None
height = None


def load_pattern():
    print("LOAD_PATTERN")
    global pattern, pattern_predict
    with open(CONST.ABS_PATH + 'dataset.csv', 'r') as csvfile:
        csvfile.readline()
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            pattern.append([int(v) for v in row[:-1]])
            pattern_predict.append(int(row[-1]))


def matching(var):
    # print("MATCHING")
    for (p, res) in zip(pattern, pattern_predict):
        if var == p:
            return res
    return None


def get_circle_in_frame(mask):
    print("GET_CIRCLE_IN_FRAME")
    area_ratio_expected = 0.6
    circles = []
    radius_list = []

    _, contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    for cnt in contours:
        (x, y), radius = cv.minEnclosingCircle(cnt)
        area_real = math.pi * radius * radius
        area_cnt = cv.contourArea(cnt)
        area_ratio = area_cnt / area_real
        if area_cnt <= 20 or area_ratio < area_ratio_expected:
            continue
        x, y, radius = int(x), int(y), int(radius)
        circles.append([x, y, radius])
        radius_list.append(radius)

    if not len(radius_list):
        return False, 0, 0
    radius_avg = np.average(radius_list)

    return True, circles, radius_avg


def may_be_dice(mask):
    # print("MAY_BE_DICE")
    result = get_point(mask)
    is_one = np.count_nonzero(result)
    if is_one >= 2:
        return True, result
    else:
        return False, None


def check_in_frame(top, left, bottom, right):
    global width, height
    if (0 <= top < height or     
        0 <= bottom < height or
        0 <= left < width or
        0 <= right < width):
        return True
    return False

def get_region(dice_size, x, y, radius):
    # print("GET_REGION")
    a, b = 4, 14
    region = []
    top = int(y - radius - int(0.5 * radius)) - a
    left = int(x - radius - int(0.5 * radius)) - a
    bottom = top + dice_size + b
    right = left + dice_size + b
    region.append([top, left, bottom, right])

    bottom = int(y + radius + int(0.5 * radius)) + a
    right = int(x + radius + int(0.5 * radius)) + a
    top = bottom - dice_size - b
    left = right - dice_size - b
    region.append([top, left, bottom, right])

    return region


def get_dice_position(mask, circles, radius_avg):
    # print("GET_DICE_POSITION")
    dice_size = int(radius_avg * CONST.SIDE_PER_RADIUS)
    data_list = []

    for cir in circles:
        x, y, radius = cir
        region_list = get_region(dice_size, x, y, radius)
        x, y, radius = int(x), int(y), int(radius)
        for region in region_list:
            top, left, bottom, right = region
            center = (int((left + right) / 2), int((top + bottom) / 2))
            roi = mask.copy()[top:bottom, left:right]
            try:
                roi = cv.resize(roi, (CONST.DICE_SIZE, CONST.DICE_SIZE))
            except:
                continue
            is_dice, point = may_be_dice(roi)
            if not is_dice:
                continue
            count = np.count_nonzero(point)
            dice = matching(point[:-1])
            if dice is None:
                continue
            data_list.append([x, y, radius, dice, point, count, center])
    data_dict = remove_redundant_dice(data_list)
    return data_dict


def remove_redundant_dice(data):
    print('REMOVE_REDUNDANT')
    result = {'2': None, '5': None, '6': None}
    data = sorted(data, key=itemgetter(3))
    for d in data:
        index_dict = str(d[3])
        if result[index_dict] is None or (result[index_dict][5] < d[5] or (result[index_dict][5] == d[5] and result[index_dict][1] > d[5])):
            result[index_dict] = d

    return result


def mask_dice(img, dict):
    print('MASK_DICE')
    color = {'2': (255, 0, 0), '5': (0, 255, 0), '6': (0, 0, 255)}
    for d in dict.keys():
        if dict[d] is None:
            continue
        x, y, radius, dice, point, count, center = dict[d]
        cv.circle(img, center, radius, color[d], -1)
        cv.circle(img, center, radius*9, color[d], 2)
    return img


def run(img):
    global width, height
    load_pattern()
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    (height, width) = gray.shape
    equ = cv.equalizeHist(gray)
    _, mask = cv.threshold(equ, 30, 255, cv.THRESH_BINARY_INV)
    status, circles, radius_avg = get_circle_in_frame(mask)
    if not status:
        return img
    print("BEFORE_GET_DICE_POSITION")
    dice_dict = get_dice_position(mask, circles, radius_avg)
    img = mask_dice(img, dice_dict)
    return img

# if __name__ == '__main__':
    # main()
    # matching()
