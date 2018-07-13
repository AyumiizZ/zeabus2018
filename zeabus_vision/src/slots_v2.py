#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage
from zeabus_vision.msg import vision_slots
from zeabus_vision.srv import vision_srv_slots
from vision_lib import *
import color_text
img_top = None
img_top_res = None
img_bot = None
img_bot_res = None
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
    if task == 'yellow_hole':
        return find_yellow_hole()
    elif task == 'red_hole':
        if req == 'big':
            find_red_hole('big')
        elif req == 'small':
            find_red_hole('small')
    elif task == 'handle':
        return find_handle()


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


def message(cx=-1, cy=-1, area=-1, appear=False):
    m = vision_slots()
    m.cx = cx
    m.cy = cy
    m.area = area
    m.appear = appear
    print(m)
    return m


def get_object(img, color):
    """
        get mask from picture and remove some noise
        Returns:
            mask (ONLY obj(args) area)
    """
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


# def get_cx_hole(mask):
#     """
#         get cx, cy and area of object
#         Returns:
#             float: cx
#             float: cy
#             float: area
#     """
#     cx = 0
#     cy = 0
#     area = 0
#     appear = False
#     global img_res
#     himg, wimg = img.shape[:2]
#     x, y, w, h = cv.boundingRect(mask)
#     top_excess = (y < 0.05*himg)
#     bot_excess = ((y+h) > 0.95*himg)
#     right_excess = (x+w > 0.95*wimg)
#     left_excess = (x < 0.05*wimg)
#     window_excess = top_excess or bot_excess or right_excess or left_excess
#     if (not window_excess):
#         cv.rectangle(img_res, (x, y), (x+w, y+h), (0, 255, 0), 2)
#         cnt = cv.findContours(mask, cv.RETR_EXTERNAL,
#                               cv.CHAIN_APPROX_SIMPLE)[1]
#         area_out = max(cnt, key=cv.contourArea)
#         copy = img_res.copy()
#         slot = copy[y:y+h, x:x+w]
#         slot = not(slot)
#         x_s, y_s, w_s, h_s = cv.boundingRect(slot)
#         cv.rectangle(img_res, (x_s, y_s), (x_s+w_s, y_s+h_s), (0, 255, 0), 2)
#         cnt = cv.findContours(slot, cv.RETR_EXTERNAL,
#                               cv.CHAIN_APPROX_SIMPLE)[1]
#         area_in = max(cv.cnt, key=cv.contourArea)
#         if area_out/area_in*100 >= 25 and area_out/area_in*100 <= 75:
#             appear = True
#             cx = x_s + (w_s/2)
#             cy = y_s + (h_s/2)
#             cv.circle(img_res, (cx, cy), 5, (0, 0, 255), -1)
#             cx = Aconvert(cx, wimg)
#             cy = -1.0*Aconvert(cy, himg)
#             area = (1.0*w_s*h_s)/(wimg*himg)
#     return cx, cy, area, appear

def get_ROI_hole(mask):
    himg, wimg = mask.shape[:2]
    ROI = []
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    for cnt in contours:
        if cv.contourArea(cnt) < 100:
            continue
        x, y, w, h = cv.boundingRect(cnt)
        mask_crop = mask[y:y+h, x:x+w]
        not_mask_crop = cv.bitwise_not(mask_crop)
        mask_area = cv.countNonZero(mask_crop)
        not_mask_area = cv.countNonZero(not_mask_crop)
        percent_not_mask = float(not_mask_area)/float(mask_area + not_mask_area)
        top_excess = (y < 0.05*himg)
        bot_excess = ((y+h) > 0.95*himg)
        right_excess = ((x+w) > 0.95*wimg)
        left_excess = (x < 0.05*wimg)
        window_excess = top_excess or bot_excess or right_excess or left_excess
        if not window_excess and (percent_not_mask < 0.75 and percent_not_mask > 0.25):
            ROI.append(cnt)
    return ROI


def find_red_hole(size):
    global img_top,img_top_res
    while img_top is None and not rospy.is_shutdown():
        img_is_none()
    mask = get_object(img=img_top,color="red")
    ROI = get_ROI_hole(mask)
    
    ROI = sorted(ROI,key=cv.contourArea)

    if ROI == []:
        mode = 1
        print_result("NOT FOUND", color_text.RED)
    elif len(ROI) == 1:
        mode = 2
        hole = ROI[0]
        print_result("FOUND ONE HOLE", color_text.CYAN)
    elif len(ROI) == 2:
        mode = 3
        ROI = ROI[-2:]
        big_hole = ROI[0]
        small_hole = ROI[1]
        print_result("FOUND TWO HOLE",color_text.GREEN)
    elif len(ROI) > 2:
        mode = 4
        ROI = ROI[-2:]
        big_hole = ROI[0]
        small_hole = ROI[1]
        print_result("FOUND BUT HAVE SOME NOISE", color_text.YELLOW)

        



def find_yellow_hole():
    pass


# def find_hole(color):
#     global img, img_res
#     while img is None and not rospy.is_shutdown():
#         img_is_none()

#     mask = get_object(color)
#     cx, cy, area, appear = get_cx_hole(mask)
#     if not(appear):
#         mode = 1
#         print_result("NOT FOUND", color_text.RED)
#     else:
#         mode = 2
#         print_result("FOUND HOLE", color_text.GREEN)

#     if mode == 1:
#         publish_result(img_res, 'bgr', pub_topic + 'img_res')
#         publish_result(mask, 'gray', pub_topic + 'mask')
#         return message(cx, cy, area, appear)
#     elif mode == 2:
#         publish_result(img_res, 'bgr', pub_topic + 'img_res')
#         publish_result(mask, 'gray', pub_topic + 'mask')
#         return message(cx, cy, area, appear)


def get_ROI_handle(mask):
    himg, wimg = mask.shape[:2]
    ROI = []
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    for cnt in contours:
        if cv.contourArea(cnt) < 100:
            continue
        x, y, w, h = cv.boundingRect(cnt)
        mask_crop = mask[y:y+h, x:x+w]
        not_mask_crop = cv.bitwise_not(mask_crop)
        mask_area = cv.countNonZero(mask_crop)
        not_mask_area = cv.countNonZero(not_mask_crop)
        percent_mask = float(mask_area)/float(mask_area + not_mask_area)
        top_excess = (y < 0.05*himg)
        bot_excess = ((y+h) > 0.95*himg)
        right_excess = ((x+w) > 0.95*wimg)
        left_excess = (x < 0.05*wimg)
        window_excess = top_excess or bot_excess or right_excess or left_excess
        if not window_excess and percent_mask > 0.9:
            ROI.append(cnt)
    return ROI


def find_handle():
    global img_bot, img_bot_res
    while img_bot is None and not rospy.is_shutdown():
        img_is_none()
    himg, wimg = img_bot.shape[:2]
    mask = get_object(img=img_bot, color="yellow")
    ROI = get_ROI_handle(mask)
    if ROI == []:
        mode = 1
        print_result("NOT FOUND", color_text.RED)
    elif len(ROI) == 1:
        mode = 2
        handle = ROI[0]
        print_result("FOUND HANDLE", color_text.GREEN)
    elif len(ROI) > 1:
        mode = 2
        handle = max(ROI, key=cv.contourArea)
        print_result("FOUND BUT HAVE SOME NOISE", color_text.YELLOW)

    if mode == 1:
        publish_result(img_bot_res, 'bgr', pub_topic + 'handle/result')
        publish_result(mask, 'gray', pub_topic + 'handle/mask')
        return message()
    elif mode == 2:
        x, y, w, h = cv.boundingRect(handle)
        cv.rectangle(img_bot_res, (x, y), (x+w, y+h), (0, 0, 255))
        area = 1.0*w*h/(wimg*himg)
        cx = x+(w/2)
        cy = y+(h/2)
        cv.circle(img_bot_res, (cx, cy), 3, (255, 0, 0), -1)
        cx = Aconvert(cx, wimg)
        cy = -1.0*Aconvert(cy, himg)
        publish_result(img_bot_res, 'bgr', pub_topic + 'handle/result')
        publish_result(mask, 'gray', pub_topic + 'handle/mask')
        return message(cx=cx, cy=cy, area=area, appear=True)


if __name__ == '__main__':
    rospy.init_node('vision_slots', anonymous=False)
    print_result("INIT NODE", color_text.GREEN)
    front_topic = get_topic("front", world)
    bottom_topic = get_topic("bottom", world)
    rospy.Subscriber(front_topic, CompressedImage, image_top_callback)
    rospy.Subscriber(bottom_topic, CompressedImage, image_bot_callback)
    print_result("INIT SUBSCRIBER", color_text.GREEN)
    rospy.Service('vision_slots', vision_srv_slots(),
                  mission_callback)
    print_result("INIT SERVICE", color_text.GREEN)
    rospy.spin()
    print_result("END PROGRAM", color_text.RED+color_text.YELLOW_HL)
    # while True and not rospy.is_shutdown():
    #     find_slots("yellow")
