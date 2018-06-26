#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage
from zeabus_vision.msg import vision_buy_a_gold_chip
from zeabus_vision.srv import vision_srv_buy_a_gold_chip
from vision_lib import *
img = None
img_res = None
sub_sampling = 1
pub_topic = "/vision/buy_a_gold_chip/"
world = "real"


def mission_callback(msg):
    """
        When call service it will run this 
        Returns:
            a group of process value from this program
    """
    print_result('mission_callback')
    task = msg.task.data
    req = msg.req.data
    print('task:', str(task))
    if task == 'buy_a_gold_chip':
        if req == 'front' :
            return find_plate()
        if req == 'bottom' :
            return find_chip()

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
    resized = cv.resize(img, dim, interpolation = cv.INTER_AREA)
    img = resized
    img_res = img.copy()

def message(cx=-1, cy=-1, hit=-1, area=-1, appear=False):
    m = vision_buy_a_gold_chip()
    m.cx = cx
    m.cy = cy
    m.area = area
    m.hit = hit
    m.appear = appear
    print(m)
    return m

def get_object():
    global img
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # sim
    # lower = np.array([0, 120, 0], dtype=np.uint8)
    # upper = np.array([37, 255, 255], dtype=np.uint8)
    # # real world
    lower = np.array([0, 92, 24], dtype=np.uint8)
    upper = np.array([60, 255, 201], dtype=np.uint8)
    mask = cv.inRange(hsv, lower, upper)
    # kernel = np.ones((5, 5), dtype=np.uint8)
    # mask = cv.GaussianBlur(mask, (5, 5), 0)
    # mask = cv.erode(mask, kernel)
    # mask = cv.erode(mask, kernel)
    # mask = cv.dilate(mask, kernel)
    # mask = cv.dilate(mask, kernel)
    return mask

def get_cx() :
    global img
    himg,wimg = img.shape[:2]
    ROI_cx = -1
    ROI_cy = -1
    area = -1
    appear = False
    hit = 0
    cnt = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    for i in cnt :
        hit += cv.contourArea(i) 
    if cnt > 1 :
        cnt = max(cnt,key=cv.contourArea)
        area = cv.contourArea(cnt)
        if area > 500 :
            x,y,w,h = cv.boundingRect(cnt)
            cv.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            cx = x + (w/2)
            cy = y + (h/2)
            top_excess = (y < 0.05*himg)
            bot_excess = ((y+h) > 0.95*himg)
            right_excess = (x+w > 0.95*wimg)
            left_excess = (x < 0.05*wimg)
            w_h_ratio = 1.0*w/h
            window_excess = top_excess or bot_excess or right_excess or left_excess
            if (not window_excess) and w_h_ratio >= 0.5 and w_h_ratio < 2:
                appear = True
                cv.circle(img_res, (cx, cy), 2, (0, 0, 255), -1)
                return cx , cy , area , hit , appear 
    return cx , cy , area , hit , appear

def get_plate() :
    global img, img_res
    while img is None and not rospy.is_shutdown():
        print('img is none.\nPlease check topic name or check camera is running')

    mask = get_object()
    cx, cy, area ,hit, appear = get_cx(mask)
    if cx == -1 and cy == -1 :
        mode = 1
    elif cx != -1 and cy != -1:
        mode = 2
    if mode == 1:
        print_result("MODE 1: CANNOT FIND PLATE")
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message()
    elif mode == 2:
        print_result("MODE 2: CAN FIND 1 CX AND 1 CY")
        himg, wimg = img.shape[:2]
        return_cx = Aconvert(cx,wimg)
        return_cy = Aconvert(cy,himg)
        return_area = (1.0*area)/(himg*wimg)
        return_his = hit/(himg*wimg)
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx=return_cx, cy=return_cy, hit = return_hit , area=return_area, appear=True)

if __name__ == '__main__':
    rospy.init_node('node_buy_a_gold_chip', anonymous=False)
    print_result("INIT NODE")

    # sim
    # TOPIC = "/syrena/bottom_cam/image_raw/compressed"
    # real world
    image_topic = get_topic("front",world)
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")
    rospy.Service('vision_buy_a_gold_chip', vision_srv_buy_a_gold_chip(),
                  mission_callback)
    print_result("INIT SERVICE")
    rospy.spin()
    print_result("END PROGRAM")




















