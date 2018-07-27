#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage
from zeabus_vision.msg import vision_slots
from zeabus_vision.srv import vision_srv_slots
from vision_lib import *
import color_text
img = None
img_res = None
sub_sampling = 1
pub_topic = "/vision/slots/"
world = "real"


def mission_callback(msg):
    """
        When call service it will run this
        Returns:
            a group of process value from this program
    """
    print_result('mission_callback', color_text.CYAN)
    task = msg.task.data
    req = msg.req.data
    print('task:', str(task))
    if task == 'hole':
        if req == 'yellow':
            return find_hole('yellow')
        if req == 'red':
            return find_hole('red')
    elif task == 'handle' :
        return find_handle()

def image_callback(msg):
    """
        Convert data from camera to image
    """
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    img = cv.resize(img, (640, 480))
    img_res = img.copy()


def message(cx=-1, cy=-1, area=-1, appear=False):
    m = vision_slots()
    m.cx = cx
    m.cy = cy
    m.area = area
    m.appear = appear
    print(m)
    return m


def get_object(color):
    """
        get mask from picture and remove some noise
        Returns:
            mask (ONLY obj(args) area)
    """
    global img
    if color == "yellow":
        if world == "real":
            hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            lower = np.array([0, 92, 24], dtype=np.uint8)
            upper = np.array([60, 255, 201], dtype=np.uint8)
            mask = cv.inRange(hsv, lower, upper)
        elif world == "sim":
            lower = np.array([0, 240, 240], dtype=np.uint8)
            upper = np.array([10, 255, 255], dtype=np.uint8)
            mask = cv.inRange(img, lower, upper)
    elif color == "red":
        if world == "real":
            hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            lower1 = np.array([0, 85, 12], dtype=np.uint8)
            upper1 = np.array([11, 234, 234], dtype=np.uint8)
            lower2 = np.array([158, 85, 12], dtype=np.uint8)
            upper2 = np.array([180, 234, 234], dtype=np.uint8)
            mask1 = cv.inRange(hsv, lower1, upper1)
            mask2 = cv.inRange(hsv, lower2, upper2)
            mask = cv.bitwise_or(mask1, mask2)
        if world == "sim":
            hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            lower1 = np.array([0, 85, 12], dtype=np.uint8)
            upper1 = np.array([11, 234, 234], dtype=np.uint8)
            lower2 = np.array([158, 85, 12], dtype=np.uint8)
            upper2 = np.array([180, 234, 234], dtype=np.uint8)
            mask1 = cv.inRange(hsv, lower1, upper1)
            mask2 = cv.inRange(hsv, lower2, upper2)
            mask = cv.bitwise_or(mask1, mask2)
    return mask

def get_cx_hole(mask):
    """
        get cx, cy and area of object
        Returns:
            float: cx
            float: cy
            float: area
    """
    cx = 0
    cy = 0
    area = 0
    appear = False
    global img_res
    himg, wimg = img.shape[:2]
    x, y, w, h = cv.boundingRect(mask)
    top_excess = (y < 0.05*himg)
    bot_excess = ((y+h) > 0.95*himg)
    right_excess = (x+w > 0.95*wimg)
    left_excess = (x < 0.05*wimg)
    window_excess = top_excess or bot_excess or right_excess or left_excess
    if (not window_excess) :
        cv.rectangle(img_res, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cnt = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        area_out = max(cnt, key=cv.contourArea)
        copy = img_res.copy()
        slot = copy[y:y+h, x:x+w]
        slot = not(slot)
        x_s, y_s, w_s, h_s = cv.boundingRect(slot)
        cv.rectangle(img_res, (x_s, y_s), (x_s+w_s, y_s+h_s), (0, 255, 0), 2)
        cnt = cv.findContours(slot, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        area_in = max(cv.cnt, key=cv.contourArea)
        if area_out/area_in*100 >= 25 and area_out/area_in*100 <= 75:
            appear = True
            cx = x_s + (w_s/2)
            cy = y_s + (h_s/2)
            cv.circle(img_res, (cx, cy), 5, (0, 0, 255), -1)
    # (xC, yC), radius = cv.minEnclosingCircle(cnt)
    # center = (int(xC), int(yC))
    # radius = int(radius)
    # cv.circle(img_res, center, radius, (255, 0, 0), 2)
            cx = Aconvert(cx, wimg)
            cy = -1.0*Aconvert(cy, himg)
            area = (1.0*w_s*h_s)/(wimg*himg)
    return cx, cy, area , appear

def get_cx_handle(mask):
    """
        get cx, cy and area of object
        Returns:
            float: cx
            float: cy
            float: area
    """
    cx = 0
    cy = 0
    area = 0
    appear = False
    global img_res
    himg, wimg = img.shape[:2]
    x, y, w, h = cv.boundingRect(mask)
    top_excess = (y < 0.05*himg)
    bot_excess = ((y+h) > 0.95*himg)
    right_excess = (x+w > 0.95*wimg)
    left_excess = (x < 0.05*wimg)
    window_excess = top_excess or bot_excess or right_excess or left_excess
    if (not window_excess) :
        cv.rectangle(img_res, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cnt = max(cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1],key = cv.contourArea)
        area = cv.contourArea(cnt)
        if area >= 100 :
            appear = True
            cx = x + (w/2)
            cy = y + (h/2)
            cv.circle(img_res, (cx, cy), 5, (0, 0, 255), -1)
            cx = Aconvert(cx, wimg)
            cy = -1.0*Aconvert(cy, himg)
    return cx , cy , area , appear

def find_hole(color):
    global img, img_res
    while img is None and not rospy.is_shutdown():
        img_is_none()

    mask = get_object(color)
    cx, cy, area ,appear = get_cx_hole(mask)
    if not(appear) :
        mode = 1
        print_result("NOT FOUND", color_text.RED)
    else:
        mode = 2
        print_result("FOUND HOLE", color_text.GREEN)

    if mode == 1:
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx,cy,area,appear)
    elif mode == 2:
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx,cy,area,appear)

def find_handle():
    global img, img_res
    while img is None and not rospy.is_shutdown():
        img_is_none()

    mask = get_object("yellow")
    cx, cy, area ,appear = get_cx_handle(mask)
    if not(appear) :
        mode = 1
        print_result("NOT FOUND", color_text.RED)
    else:
        mode = 2
        print_result("FOUND HANDLE", color_text.GREEN)

    if mode == 1:
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx,cy,area,appear)
    elif mode == 2:
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx,cy,area,appear)

if __name__ == '__main__':
    rospy.init_node('vision_slots', anonymous=False)
    print_result("INIT NODE", color_text.GREEN)
    image_topic = get_topic("front", world)
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    image_topic = get_topic("bottom", world)
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER", color_text.GREEN)
    rospy.Service('vision_slots', vision_srv_slots(),
                  mission_callback)
    print_result("INIT SERVICE", color_text.GREEN)
    rospy.spin()
    print_result("END PROGRAM", color_text.RED+color_text.YELLOW_HL)
    # while True and not rospy.is_shutdown():
    #     find_slots("yellow")
