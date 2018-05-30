#!/usr/bin/python2.7
"""
    To use in simulator please assign world varible (line 20) to 'sim' 
    # world = 'sim'
    To use in real world please assign world varible (line 20) to 'real' 
    # world = 'real'
"""
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from zeabus_vision.msg import vision_qualifying_marker
from zeabus_vision.srv import vision_srv_qualifying_marker
from cv_bridge import CvBridge, CvBridgeError
from vision_lib import *
img = None
img_res = None
sub_sampling = 1
pub_topic = "/vision/qualifying_marker/"
world = "real"


def mission_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    print_result('mission_callback')

    task = msg.task.data

    print('task:', str(task))
    if task == 'marker':
        return find_marker()


def image_callback(msg):
    """
        Convert data from camera to image
    """
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    size = 500
    r = 1.0*size / img.shape[1]
    dim = (size, int(img.shape[0] * r))
    resized = cv.resize(img, dim, interpolation=cv.INTER_AREA)
    img = resized
    img_res = img.copy()


def message(cx_left=-1, cx_right=-1, area=-1, appear=False):
    """
        Convert value into a message (from vision_path.msg)
        Returns:
            vision_marker (message): a group of value from args
    """
    m = vision_qualifying_marker()
    m.cx_left = cx_left
    m.cx_right = cx_right
    m.area = area
    m.appear = appear
    print(m)
    return m


def get_object():
    """
        get mask from picture and remove some noise
        Returns:
            mask (ONLY MARKER AREA)
    """
    global img
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # real world
    # lower = np.array([0, 0, 0], dtype=np.uint8)
    # upper = np.array([180, 180, 68], dtype=np.uint8)

    lower, upper = get_color("qualifying", "orange", world)
    mask = cv.inRange(hsv, lower, upper)
    kernel = np.ones((31, 3), dtype=np.uint8)
    mask = cv.GaussianBlur(mask, (5, 5), 0)
    mask = cv.erode(mask, kernel)
    mask = cv.erode(mask, kernel)
    mask = cv.dilate(mask, kernel)
    mask = cv.dilate(mask, kernel)
    return mask

def find_marker():
    """
        find marker on the picture draw a line
        on left side and right side of marker

        Returns:
            message: (a group of data from vision_path.msg)
            if cannot find marker:
                return default value 
            else if can find marker:
                return float: cx_left (left side of marker) in range of -1 to 1
                       float: cx_right (right side of marker) in range of -1 to 1
                       float: area in range of 0 to 1
                       bool: appear is true
    """
    global img, img_res
    while img is None and not rospy.is_shutdown():
        print('img is none.\nPlease check topic name or check camera is running')
    himg, wimg = img.shape[:2]
    print (himg, wimg)
    if wimg != 500:
        return message()

    mask = get_object()
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    ROI = []
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > 1500:
            x,y,w,h = cv.boundingRect(cnt)
            top_excess = (y < 0.05*himg)
            bot_excess = ((y+h) > 0.95*himg)
            w_ratio = 0.2
            if top_excess and bot_excess and 1.0*w/wimg < w_ratio:
                ROI.append(cnt)

                
    if len(ROI) == 0:
        mode = 1
    elif len(ROI) == 1:
        mode = 2
    elif len(ROI) > 1:
        mode = 3

    if mode == 1:
        print_result("MODE 1: CANNOT FIND MARKER")
        publish_result(img, 'bgr', pub_topic + 'img')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message()
    elif mode == 2:
        print_result("MODE 2: CAN FIND MARKER")
        cnt = ROI[0]
        x, y, w, h = cv.boundingRect(cnt)
        cx_left = x
        cx_right = x+w
        cv.line(img, (cx_left, 0), (cx_left, himg), (255, 0, 0), 1)
        cv.putText(img, "left", (x+5, himg-30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,
                   [0, 0, 0])
        cv.line(img, (cx_right, 0), (cx_right, himg), (255, 0, 0), 1)
        cv.putText(img, "right", (x+w+5, himg-30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,
                   [0, 0, 0])

        cx_left = Aconvert(cx_left, wimg)
        cx_right = Aconvert(cx_right, wimg)
        area = (1.0*area)/(himg*wimg)
        publish_result(img, 'bgr', pub_topic + 'img')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx_left=cx_left, cx_right=cx_right, area=area, appear=True)
    elif mode == 3:
        print_result("MODE 3: MAYBE A LOT OF NOISE")
        cnt = max(ROI,key=cv.contourArea)
        cx_left = x
        cx_right = x+w
        cv.line(img, (cx_left, 0), (cx_left, himg), (255, 0, 0), 1)
        cv.putText(img, "left", (x+5, himg-30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,
                   [0, 0, 0])
        cv.line(img, (cx_right, 0), (cx_right, himg), (255, 0, 0), 1)
        cv.putText(img, "right", (x+w+5, himg-30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,
                   [0, 0, 0])

        cx_left = Aconvert(cx_left, wimg)
        cx_right = Aconvert(cx_right, wimg)
        area = (1.0*area)/(himg*wimg)
        publish_result(img, 'bgr', pub_topic + 'img')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx_left=cx_left, cx_right=cx_right, area=area, appear=True)

# an old buggy one
# def find_marker():
#     """
#         find marker on the picture draw a line
#         on left side and right side of marker

#         Returns:
#             message: (a group of data from vision_path.msg)
#             if cannot find marker:
#                 return default value 
#             else if can find marker:
#                 return float: cx_left (left side of marker) in range of -1 to 1
#                        float: cx_right (right side of marker) in range of -1 to 1
#                        float: area in range of 0 to 1
#                        bool: appear is true
#     """
#     global img, img_res
#     while img is None and not rospy.is_shutdown():
#         print('img is none.\nPlease check topic name or check camera is running')

#     himg, wimg = img.shape[:2]
#     print (himg, wimg)
#     mask = get_object()
#     contours = cv.findContours(
#         mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
#     if len(contours) == 0:
#         mode = 1
#     elif len(contours) >= 1:
#         cnt = max(contours, key=cv.contourArea)
#         area = cv.contourArea(cnt)
#         if area < 3000:
#             mode = 1
#         elif area >= 3000:
#             mode = 2

#     if mode == 1:
#         print_result("MODE 1: CANNOT FIND MARKER")
#         publish_result(img, 'bgr', pub_topic + 'img')
#         publish_result(mask, 'gray', pub_topic + 'mask')
#         return message()
#     elif mode == 2:
#         print_result("MODE 2: CAN FIND MARKER")
#         x, y, w, h = cv.boundingRect(cnt)
#         cx_left = x
#         cx_right = x+w
#         cv.line(img, (cx_left, 0), (cx_left, himg), (255, 0, 0), 1)
#         cv.putText(img, "left", (x+5, himg-30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,
#                    [0, 0, 0])
#         cv.line(img, (cx_right, 0), (cx_right, himg), (255, 0, 0), 1)
#         cv.putText(img, "right", (x+w+5, himg-30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,
#                    [0, 0, 0])

#         cx_left = Aconvert(cx_left, wimg)
#         cx_right = Aconvert(cx_right, wimg)
#         area = (1.0*area)/(himg*wimg)
#         publish_result(img, 'bgr', pub_topic + 'img')
#         publish_result(mask, 'gray', pub_topic + 'mask')
#         return message(cx_left=cx_left, cx_right=cx_right, area=area, appear=True)


if __name__ == '__main__':
    rospy.init_node('vision_qualifying_marker', anonymous=True)
    print_result("INIT NODE")
    image_topic = get_topic("front", world)
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")
    rospy.Service('vision_qualifying_marker',
                  vision_srv_qualifying_marker(), mission_callback)
    print_result("INIT SERVICE")
    rospy.spin()
    print_result("END PROGRAM")
