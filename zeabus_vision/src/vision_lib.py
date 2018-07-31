#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
import constant as CONST
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import color_text


def range_str2list(str):
    str = str.split(',')
    return np.array([int(str[0]), int(str[1]), int(str[2])], np.uint8)


def get_color_range(color, camera_position, number, mission):
    lower = None
    upper = None
    color_list = CONST.COLOR_LIST
    if color in color_list:
        lower = rospy.get_param(
            'color_range_' + str(camera_position) + '_' + str(number) + '_' + str(mission) + '/color_' + camera_position + '/lower_' + color, '0,0,0')
        upper = rospy.get_param(
            'color_range_' + str(camera_position) + '_' + str(number) + '_' + str(mission) + '/color_' + camera_position + '/upper_' + color, '179,255,255')
        lower = range_str2array(lower)
        upper = range_str2array(upper)
        print "FOUND"
    print(lower, upper)
    return lower, upper

def img_is_none():
    print(color_text.RED + 'img is none.\nPlease check topic name or check camera is running' + color_text.DEFAULT)


def print_result(msg, color=color_text.DEFAULT):
    """
        print ('<----------') + str(msg) + ('---------->')
        #len of <---msg---> = 50
    """
    print '<{:-^50}>'.format(' ' + color + str(msg) + color_text.DEFAULT + ' ')


def publish_result(img, type, topicName):
    """
        publish picture
    """
    if img is None:
        img = np.zeros((200, 200))
        type = "gray"
    bridge = CvBridge()
    pub = rospy.Publisher(
        str(topicName), Image, queue_size=10)
    if type == 'gray':
        msg = bridge.cv2_to_imgmsg(img, "mono8")
    elif type == 'bgr':
        msg = bridge.cv2_to_imgmsg(img, "bgr8")
    pub.publish(msg)


def get_color(task, color, world):
    """
        get range of each color
        Returns:
            np.array(uint8): lower bound of args color
            np.array(uint8): upper bound of args color
    """
    lower = None
    upper = None
    if task == "qualifying":
        if color == "orange":
            if world == "real":
                lower = np.array([0, 0, 0], dtype=np.uint8)
                upper = np.array([180, 180, 68], dtype=np.uint8)
    if task == "path":
        if color == "yellow":
            if world == "real":
                lower = np.array([20, 120, 0], dtype=np.uint8)
                upper = np.array([62, 255, 255], dtype=np.uint8)
            elif world == "sim":
                lower = np.array([0, 120, 0], dtype=np.uint8)
                upper = np.array([37, 255, 255], dtype=np.uint8)
    return lower, upper


def get_topic(cam, world):
    topic = None
    if cam == "front":
        if world == "real":
            topic = "/stereo/right/image_raw/compressed"
        if world == "sim":
            topic = "/syrena/front_cam/image_raw/compressed"
    # if cam == "front":
    #     if world == "real":
    #         topic = "/stereo/right/image_rect_color/compressed"
    #     if world == "sim":
    #         topic = "/syrena/front_cam/image_raw/compressed"
    if cam == "bottom":
        if world == "real":
            topic = "/bottom/left/image_raw/compressed"
        if world == "sim":
            topic = "/syrena/bottom_cam/image_raw/compressed"
    return topic


def range_str2array(string):
    string = string.split(',')
    return np.array([int(string[0]), int(string[1]), int(string[2])], dtype=np.uint8)




def Aconvert(inp, full):
    """
        Convert cx cy to int in range of -1 to 1
        Returns:
            float: result
    """
    inp = float(inp)
    full = float(full)
    res = (inp - (full / 2.0)) / (full / 2.0)
    return res


def Oreintation(moment):
    tmp = pow((moment['mu20'] - moment['mu02']), 2)
    tmp = math.sqrt(tmp + (4 * pow(moment['mu11'], 2)))

    k = moment['mu02'] - moment['mu20']

    l = 2 * moment['mu11']

    rad_maj = math.atan2(k + tmp, l)
    rad_min = math.atan2(k - tmp, l)
    return rad_maj, rad_min


def nothing(variable):
    pass


def get_mode(data):
    data = np.array(data)
    if len(data.shape) > 1:
        data = data.ravel()
    try:
        mode = statistics.mode(data)
    except ValueError:
        mode = None
    return mode


def get_cv(data):
    mean = cv.mean(data)[0]
    sd = cv.meanStdDev(data, mean)[0]
    return sd / mean


def shrinking(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)
    hmin = h.min()
    hmax = h.max()
    hl = 180
    hs = ((h - hmin) / (hmax - hmin)) * (hl - 1)

    smin = s.min()
    smax = s.max()
    sl = 255
    ss = ((s - smin) / (smax - smin)) * (sl - 1)

    vmin = v.min()
    vmax = v.max()
    vl = 255
    vs = ((v - vmin) / (vmax - vmin)) * (vl - 1)

    result = cv.merge((hs, ss, vs))
    return cv.cvtColor(result, cv.COLOR_HSV2BGR)


def adjust_gamma(imgBGR=None, gamma=1):
    if imgBGR is None:
        print('given value to imgBGR argument\n' +
              'adjust_gamma_by_value(imgBGR, gamma)')

    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) *
                      255 for i in np.arange(0, 256)]).astype("uint8")

    return cv.LUT(imgBGR, table)


def adjust_gamma_by_v(imgBGR=None):
    if imgBGR is None:
        print('given value to imgBGR argument\n' +
              'adjust_gamma_by_value(imgBGR)')
        return

    hsv = cv.cvtColor(imgBGR, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)
    vMean = cv.mean(v)[0]
    gamma = vMean / 13
    # print 'gamma : ' + str(gamma)

    if gamma == 0:
        g = 1.0
    else:
        g = gamma / 10.0
    invGamma = 1.0 / g
    table = []
    for i in np.arange(0, 256):
        table.append(((i / 255.0) ** invGamma) * 255)
    table = np.array(table).astype('uint8')
    res = cv.LUT(imgBGR, table)
    return res


def get_kernel(shape='rect', ksize=(5, 5)):
    if shape == 'rect':
        return cv.getStructuringElement(cv.MORPH_RECT, ksize)
    elif shape == 'ellipse':
        return cv.getStructuringElement(cv.MORPH_ELLIPSE, ksize)
    elif shape == 'plus':
        return cv.getStructuringElement(cv.MORPH_CROSS, ksize)
    else:
        return None


def color_mapping(mono):
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
    bgr = cv.applyColorMap(mono, cv.COLORMAP_HSV)
    return bgr


def crop(imgBGR, outerRow, outerCol):
    rows, cols, ch = imgBGR.shape
    innerRow = int(rows - (outerRow * 2))
    maskOuterRow = np.zeros((int(outerRow), cols)) + 255
    maskInnerRow = np.zeros((innerRow, cols))
    maskRow = np.concatenate(
        (maskOuterRow, maskInnerRow, maskOuterRow), axis=0)

    innerCol = int(cols - (outerCol * 2))
    maskOuterCol = np.zeros((rows, int(outerCol))) + 255
    maskInnerCol = np.zeros((rows, innerCol))
    maskCol = np.concatenate(
        (maskOuterCol, maskInnerCol, maskOuterCol), axis=1)

    mask = cv.add(maskRow, maskCol)
    mask = np.uint8(mask)

    sb, sg, sr = cv.split(imgBGR)

    b = cv.subtract(sb, mask)
    g = cv.subtract(sg, mask)
    r = cv.subtract(sr, mask)

    return cv.merge((b, g, r))


def crop_gray(imgGray, outerRow, outerCol):
    rows, cols = imgGray.shape
    innerRow = int(rows - (outerRow * 2))
    maskOuterRow = np.zeros((int(outerRow), cols)) + 255
    maskInnerRow = np.zeros((innerRow, cols))
    maskRow = np.concatenate(
        (maskOuterRow, maskInnerRow, maskOuterRow), axis=0)

    innerCol = int(cols - (outerCol * 2))
    maskOuterCol = np.zeros((rows, int(outerCol))) + 255
    maskInnerCol = np.zeros((rows, innerCol))
    maskCol = np.concatenate(
        (maskOuterCol, maskInnerCol, maskOuterCol), axis=1)

    mask = cv.add(maskRow, maskCol)
    mask = np.uint8(mask)

    resGray = cv.subtract(imgGray, mask)

    return resGray


def cut_frame_top(imgBinary):
    # cut 1/4 of top of frame
    rows, cols = imgBinary.shape
    maskTop = np.zeros((int(rows / 4), cols))
    maskBottom = np.ones((int(3 * rows / 4), cols))
    res = np.concatenate((maskTop, maskBottom), axis=0)
    res = cv.bitwise_and(res, res, mask=imgBinary)
    return res


def cut_frame_bottom(imgBinary):
    # cut 1/4 of bottom of frame
    rows, cols = imgBinary.shape
    maskTop = np.ones((int(3 * rows / 4), cols))
    maskBottom = np.zeros((int(rows / 4), cols))
    res = np.concatenate((maskTop, maskBottom), axis=0)
    res = cv.bitwise_and(res, res, mask=imgBinary)
    return res


def cut_contours(M, w, h, range_w, range_h):
    cx = None
    cy = None
    try:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
    except:
        print 'err'
    if cx is None:
        return True
    if cx <= range_w or cy <= range_h or cx >= w - range_w or cy >= h - range_h:
        return True
    return False


def brightness(imgBGR, brightnessValue):
    hsv = cv.cvtColor(imgBGR, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)
    v = np.int16(v)
    v = np.clip(v + brightnessValue, 0, 255)
    v = np.uint8(v)
    hsv = cv.merge((h, s, v))
    return cv.cvtColor(hsv, cv.COLOR_HSV2BGR)


def clip_v(imgBGR, min, max):
    hsv = cv.cvtColor(imgBGR, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)
    v = np.clip(v, min, max)
    hsv = cv.merge((h, s, v))
    return cv.cvtColor(hsv, cv.COLOR_HSV2BGR)


def equalization_bgr(imgBGR):
    b, g, r = cv.split(imgBGR)
    b = cv.equalizeHist(b)
    g = cv.equalizeHist(g)
    r = cv.equalizeHist(r)
    equBGR = cv.merge((b, g, r))
    return equBGR


def equalization_hsv(imgHSV):
    h, s, v = cv.split(imgHSV)
    s = cv.equalizeHist(s)
    v = cv.equalizeHist(v)
    equHSV = cv.merge((h, s, v))
    return equHSV


def equalization_gray(imgGRAY):
    equGRAY = cv.equalizeHist(imgGRAY)

    return equGRAY


def clahe(imgBGR):
    lab = cv.cvtColor(imgBGR, cv.COLOR_BGR2Lab)
    l, a, b = cv.split(lab)
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv.merge((l, a, b))
    resBGR = cv.cvtColor(lab, cv.COLOR_Lab2BGR)
    return resBGR


def stretching_hsv(hsv):
    h, s, v = cv.split(hsv)

    if s.min() > 0:
        s *= int(round(255.0 / (s.max() - s.min())))
    if v.min() > 0:
        v *= int(round(255.0 / (v.max() - v.min())))
    hsv = cv.merge((h, s, v))
    return hsv


def stretching_bgr(bgr):
    b, g, r = cv.split(bgr)
    b -= b.min()
    b *= int(round(255.0 / (b.max() - b.min())))
    g -= g.min()
    g *= int(round(255.0 / (g.max() - g.min())))
    r -= r.min()
    r *= int(round(255.0 / (r.max() - r.min())))

    img = cv.merge((b, g, r))
    return img


def stretching(img):

    b, g, r = cv.split(img)
    b -= b.min()
    b *= int(round(255.0 / (b.max() - b.min())))
    g -= g.min()
    g *= int(round(255.0 / (g.max() - g.min())))
    r -= r.min()
    r *= int(round(255.0 / (r.max() - r.min())))

    img = cv.merge((b, g, r))
    img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(img)

    s -= s.min()
    s *= int(round(255.0 / (s.max() - s.min())))

    v -= v.min()
    v *= int(round(255.0 / (v.max() - v.min())))

    img = cv.merge((h, s, v))
    img = cv.cvtColor(img, cv.COLOR_HSV2BGR)

    return img

# def erode(imgBin, ker):
#     return cv.erode(imgBin, ker, iterations=1)


# def dilate(imgBin, ker):
#     return cv.dilate(imgBin, ker, iterations=1)


# def close(imgBin, ker):
#     return cv.morphologyEx(imgBin, cv.MORPH_CLOSE, ker)


# def open_morph(imgBin, ker):
#     return cv.morphologyEx(imgBin, cv.MORPH_OPEN, ker)

def pre_process_dice(img_bgr):
    return img_bgr


def pre_process(img_bgr, mission):
    if mission == 'shoot_craps':
        return pre_process_dice(img_bgr)
    # elif mission == ''
    #     return pre_process_xxx(img_bgr)
    else:
        return img_bgr

# def Aconvert(inp, full):
#     """
#         Convert cx cy to int in range of -1 to 1
#         Returns:
#             float: result
#     """
#     inp = float(inp)
#     full = float(full)
#     res = (inp - (full / 2.0)) / (full / 2.0)
#     return res
class Points:
    converted_cx = -1
    converted_cy = -1
    def __init__(self,cx,cy,himg=-1,wimg=-1):
        self.cx = cx
        self.cy = cy
        if wimg != -1:
            self.converted_cx = ((2.0*cx)/wimg)-1
        if himg != -1:
            self.converted_cy = -1*(((2.0*cy)/himg)-1)
