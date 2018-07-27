#!/usr/bin/python2.7
import rospy
import numpy as np
import cv2 as cv
from zeabus_vision.msg import vision_cash_in_your_chip
from zeabus_vision.srv import vision_srv_cash_in_your_chip
from cv_bridge import CvBridge, CvBridgeError
from vision_lib import *
from sensor_msgs.msg import CompressedImage, Image
img_top = None
img_top_res = None
img_bot = None
img_bot_res = None
sub_sampling = 1
pub_topic = "/vision/cash/"
world = "real"


def mission_callback(msg):
    print_result('mission_callback', color_text.CYAN)

    task = msg.task.data
    req = msg.req.data
    color = ['red', 'green', 'yellow']
    print('task: ', str(task), 'req: ', str(req))
    if task == 'cash_in' and req == 'bin':
        return find_bin()
    elif task == 'cash_in_top' and req in color:
        return find_cone_top(str(req))
    elif task == 'cash_in_bottom' and req in color:
        return find_cone_bot(str(req))


def image_top_callback(msg):
    """
        Convert data from camera to image
    """
    global img_top, sub_sampling, img_top_res
    arr = np.fromstring(msg.data, np.uint8)
    img_top = cv.resize(cv.imdecode(arr, 1), (0, 0),
                        fx=sub_sampling, fy=sub_sampling)
    himg, wimg = img_top.shape[:2]
    img_top = cv.resize(img_top, (int(wimg/3), int(himg/3)))
    img_top_res = img_top.copy()


def image_bot_callback(msg):
    """
        Convert data from camera to image
    """
    global img_bot, sub_sampling, img_bot_res
    arr = np.fromstring(msg.data, np.uint8)
    img_bot = cv.resize(cv.imdecode(arr, 1), (0, 0),
                        fx=sub_sampling, fy=sub_sampling)
    himg, wimg = img_bot.shape[:2]
    img_bot = cv.resize(img_bot, (int(wimg/3), int(himg/3)))
    img_bot_res = img_bot.copy()


def message(cx1=-1, cy1=-1, cx2=-1, cy2=-1, area=-1, mode=0):
    m = vision_cash_in_your_chip()
    m.cx1 = cx1
    m.cy1 = cy1
    m.cx2 = cx2
    m.cy2 = cy2
    m.area = area
    m.mode = mode
    print(m)
    return m


def get_object(img, obj, color):
    if obj == "bin" or obj == "cone":
        if color == "yellow":
            hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            lower = np.array([0, 92, 24], dtype=np.uint8)
            upper = np.array([60, 255, 201], dtype=np.uint8)
            mask = cv.inRange(hsv, lower, upper)
    return mask


def get_ROI(mask):
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    himg, wimg = mask.shape[:2]
    ROI = []
    for cnt in contours:
        if cv.contourArea(cnt) < 100:
            continue
        x, y, w, h = cv.boundingRect(cnt)
        top_excess = (y < 0.05 * himg)
        bot_excess = (y+h > 0.95*himg)
        left_excess = (x < 0.05*wimg)
        right_excess = (x+w > 0.95*wimg)
        window_excess = top_excess or bot_excess or left_excess or right_excess
        if not window_excess:
            ROI.append(cnt)
    return ROI


def find_bin():
    global img_bot, img_bot_res
    while img_bot is None and not rospy.is_shutdown():
        img_is_none()

    mask = get_object(img=img_bot, obj='bin', color='yellow')
    ROI = get_ROI(mask)
    ROI = sorted(ROI, key=cv.contourArea)[::1]
    mode = len(ROI)
    himg,wimg = img_bot.shape[:2]
    cv.line(img_bot_res, (int(0.5*wimg), 0),
            (int(0.5*wimg), himg), (220, 209, 255), 3)
    cv.line(img_bot_res, (0, int(0.5*himg)),
            (wimg, int(0.5*himg)), (207, 198, 207), 3)
    if mode == 0:
        print_result("CANNOT FIND BIN", color_text.RED)
        publish_result(img_bot_res, 'bgr', pub_topic + 'result')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message()
    elif mode == 1:
        print_result("CAN FIND ONLY ONE BIN", color_text.YELLOW)
        Bin1 = ROI[0]
        himg, wimg = img_bot.shape[:2]
        x, y, w, h = cv.boundingRect(Bin1)
        cv.rectangle(img_bot_res, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cx1 = x+(w/2)
        cy1 = y+(h/2)
        cv.circle(img_bot_res, (cx1, cy1), 3, (0, 0, 255), -1)
        cx1 = Aconvert(cx1, wimg)
        cy1 = -1.0*Aconvert(cy1, himg)
        area = (1.0*w*h)/(wimg*himg)
        publish_result(img_bot_res, 'bgr', pub_topic + 'result')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx1=cx1, cy1=cy1, area=area, mode=mode)

    elif mode >= 2:
        if mode == 2:
            print_result("CAN FIND TWO BINS", color_text.GREEN)
        else:
            print_result("CAN FIND BUT HAVE NOISE",
                         color_text.RED_HL+color_text.YELLOW)
        Bin1 = ROI[0]
        Bin2 = ROI[1]
        himg, wimg = img_bot.shape[:2]
        x1, y1, w1, h1 = cv.boundingRect(Bin1)
        cv.rectangle(img_bot_res, (x1, y1), (x1+w1, y1+h1), (0, 255, 0), 2)
        cx1 = x1+(w1/2)
        cy1 = y1+(h1/2)
        cv.circle(img_bot_res, (cx1, cy1), 3, (0, 0, 255), -1)
        cx1 = Aconvert(cx1, wimg)
        cy1 = -1.0*Aconvert(cy1, himg)
        area1 = (1.0*w1*h1)/(wimg*himg)
        x2, y2, w2, h2 = cv.boundingRect(Bin2)
        cv.rectangle(img_bot_res, (x2, y2), (x2+w2, y2+h2), (0, 255, 0), 2)
        cx2 = x2+(w2/2)
        cy2 = y2+(h2/2)
        cv.circle(img_bot_res, (cx2, cy2), 3, (0, 0, 255), -1)
        cx2 = Aconvert(cx2, wimg)
        cy2 = -1.0*Aconvert(cy2, himg)
        area2 = (1.0*w2*h2)/(wimg*himg)
        publish_result(img_bot_res, 'bgr', pub_topic + 'result')
        publish_result(mask, 'gray', pub_topic + 'mask')
        area = (area1+area2)/2
        return message(cx1=cx1, cy1=cy1, cx2=cx2, cy2=cy2, area=area, mode=mode)


def find_cone_top(color):
    global img_top, img_top_res
    while img_top is None and not rospy.is_shutdown():
        img_is_none()

    mask = get_object(img=img_top, obj='cone', color=color)
    ROI = get_ROI(mask)
    ROI = sorted(ROI, key=cv.contourArea)[::1]
    mode = len(ROI)
    if mode == 0:
        print_result("CANNOT FIND CONE", color_text.RED)
        publish_result(img_bot_res, 'bgr', pub_topic + 'result')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message()
    elif mode == 1:
        print_result("CAN FIND ONLY ONE CONE", color_text.YELLOW)
        cone1 = ROI[0]
        himg, wimg = img_top.shape[:2]
        x, y, w, h = cv.boundingRect(cone1)
        cv.rectangle(img_top_res, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cx1 = x+(w/2)
        cy1 = y+(h/2)
        cv.circle(img_top_res, (cx1, cy1), 3, (0, 0, 255), -1)
        cx1 = Aconvert(cx1, wimg)
        cy1 = -1.0*Aconvert(cy1, himg)
        area = (1.0*w*h)/(wimg*himg)
        publish_result(img_top_res, 'bgr', pub_topic + 'result')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx1=cx1, cy1=cy1, area=area, mode=mode)

    elif mode >= 2:
        if mode == 2:
            print_result("CAN FIND TWO CONE", color_text.GREEN)
        else:
            print_result("CAN FIND BUT HAVE NOISE",
                         color_text.RED_HL+color_text.YELLOW)
        cone1 = ROI[0]
        cone2 = ROI[1]
        himg, wimg = img_top.shape[:2]
        x1, y1, w1, h1 = cv.boundingRect(cone1)
        cv.rectangle(img_top_res, (x1, y1), (x1+w1, y1+h1), (0, 255, 0), 2)
        cx1 = x1+(w1/2)
        cy1 = y1+(h1/2)
        cv.circle(img_top_res, (cx1, cy1), 3, (0, 0, 255), -1)
        cx1 = Aconvert(cx1, wimg)
        cy1 = -1.0*Aconvert(cy1, himg)
        area1 = (1.0*w1*h1)/(wimg*himg)
        x2, y2, w2, h2 = cv.boundingRect(cone2)
        cv.rectangle(img_top_res, (x2, y2), (x2+w2, y2+h2), (0, 255, 0), 2)
        cx2 = x2+(w2/2)
        cy2 = y2+(h2/2)
        cv.circle(img_top_res, (cx2, cy2), 3, (0, 0, 255), -1)
        cx2 = Aconvert(cx2, wimg)
        cy2 = -1.0*Aconvert(cy2, himg)
        area2 = (1.0*w2*h2)/(wimg*himg)
        publish_result(img_top_res, 'bgr', pub_topic + 'result')
        publish_result(mask, 'gray', pub_topic + 'mask')
        area = (area1+area2)/2
        return message(cx1=cx1, cy1=cy1, cx2=cx2, cy2=cy2, area=area, mode=mode)


def find_cone_bot(color):
    global img_bot, img_bot_res
    while img_bot is None and not rospy.is_shutdown():
        img_is_none()

    mask = get_object(img=img_bot, obj='cone', color=color)
    ROI = get_ROI(mask)
    ROI = sorted(ROI, key=cv.contourArea)[::1]
    mode = len(ROI)

    if mode == 0:
        print_result("CANNOT FIND CONE", color_text.RED)
        publish_result(img_bot_res, 'bgr', pub_topic + 'result')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message()
    elif mode >= 1:
        if mode <= 3:
            print_result("CAN FIND BUT NOT SURE BIN OR CONE", color_text.GREEN)
        else:
            print_result("CAN FIND BUT HAVE A LOT OF NOISE", color_text.YELLOW)
        cone = ROI[0]
        himg, wimg = img_top.shape[:2]
        x, y, w, h = cv.boundingRect(cone1)
        cv.rectangle(img_top_res, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cx1 = x+(w/2)
        cy1 = y+(h/2)
        cv.circle(img_top_res, (cx1, cy1), 3, (0, 0, 255), -1)
        cx1 = Aconvert(cx1, wimg)
        cy1 = -1.0*Aconvert(cy1, himg)
        area = (1.0*w*h)/(wimg*himg)
        publish_result(img_top_res, 'bgr', pub_topic + 'result')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx1=cx1, cy1=cy1, area=area, mode=mode)


if __name__ == '__main__':
    rospy.init_node('vision_cash_in_your_chip', anonymous=False)
    print_result("INIT NODE", color_text.GREEN)
    front_topic = get_topic("front", world)
    bottom_topic = get_topic("bottom", world)
    rospy.Subscriber(front_topic, CompressedImage, image_top_callback)
    rospy.Subscriber(bottom_topic, CompressedImage, image_bot_callback)
    print_result("INIT SUBSCRIBER", color_text.GREEN)
    rospy.Service('vision_cash_in_your_chip', vision_srv_cash_in_your_chip(),
                  mission_callback)
    print_result("INIT SERVICE", color_text.GREEN)
    rospy.spin()
    print_result("END PROGRAM", color_text.RED+color_text.YELLOW_HL)
