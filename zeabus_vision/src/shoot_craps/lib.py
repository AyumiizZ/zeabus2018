'''
    File name: lib.py
    Author: zeabus2018
    Date created: 05/27/2018
    Python Version: 3.6.1
'''
import math
import numpy as np
import cv2 as cv
import constant_dice as CONST

'''
    Get index of point on dice.
     ___ ___ ___
    |_1_|_2_|_3_|
    |_4_|_5_|_6_|
    |_7_|_8_|_9_|

'''


def get_kernel(shape='rect', ksize=(5, 5)):
    if shape == 'rect':
        return cv.getStructuringElement(cv.MORPH_RECT, ksize)
    elif shape == 'ellipse':
        return cv.getStructuringElement(cv.MORPH_ELLIPSE, ksize)
    elif shape == 'plus':
        return cv.getStructuringElement(cv.MORPH_CROSS, ksize)
    else:
        return None


def clahe(imgBGR):
    lab = cv.cvtColor(imgBGR, cv.COLOR_BGR2Lab)
    l, a, b = cv.split(lab)
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv.merge((l, a, b))
    resBGR = cv.cvtColor(lab, cv.COLOR_Lab2BGR)
    return resBGR


def adjust_gamma(imgBGR=None, gamma=1):
    if imgBGR is None:
        print('given value to imgBGR argument\n' +
              'adjust_gamma_by_value(imgBGR, gamma)')

    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) *
                      255 for i in np.arange(0, 256)]).astype("uint8")

    return cv.LUT(imgBGR, table)


def color_mapping(mat):
    '''
        cv.COLORMAP_AUTUMN,
        cv.COLORMAP_BONE,
        cv.COLORMAP_JET,
        cv.COLORMAP_WINTER,
        cv.COLORMAP_RAINBOW,
        cv.COLORMAP_OCEAN,
        cv.COLORMAP_SUMMER,
        cv.COLORMAP_SPRING,
        cv.COLORMAP_COOL,
        cv.COLORMAP_HSV,
        cv.COLORMAP_PINK,
        cv.COLORMAP_HOT
    '''
    norm = None
    norm = cv.normalize(src=mat, dst=norm, alpha=0, beta=255,
                        norm_type=cv.NORM_MINMAX, dtype=cv.CV_8UC3)
    return cv.applyColorMap(norm, cv.COLORMAP_HSV)


def inRangeBGR(bgr, lowerb, upperb):
    r, c, _ = bgr.shape
    bgr = cv.split(bgr)
    result = np.zeros((r, c), np.uint8)
    result.fill(255)
    for i in range(3):
        result[bgr[i] < lowerb[i]] = 0
        result[bgr[i] > upperb[i]] = 0

    return result
