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
    print('task:', str(task) ,'req', str(req))
    if task == 'yellow_hole':
        return find_yellow_hole()
    elif task == 'red_hole':
        if req == 'big':
            return find_red_hole('big')
        elif req == 'small':
            return find_red_hole('small')
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


def message(cx=-1, cy=-1, area=-1, appear=False, mode=1):
    m = vision_slots()
    m.cx = cx
    m.cy = cy
    m.area = area
    m.appear = appear
    m.mode = mode
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
            lower = np.array([15, 146, 0], dtype=np.uint8)
            upper = np.array([62, 255, 255], dtype=np.uint8)
            mask = cv.inRange(hsv, lower, upper)
        elif world == "sim":
            lower = np.array([0, 240, 240], dtype=np.uint8)
            upper = np.array([10, 255, 255], dtype=np.uint8)
            mask = cv.inRange(img, lower, upper)
    elif color == "red":
        if world == "real":
            hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            lower1 = np.array([0, 39, 17], dtype=np.uint8)
            upper1 = np.array([32, 208, 115], dtype=np.uint8)
            lower2 = np.array([122, 39, 17], dtype=np.uint8)
            upper2 = np.array([180, 208, 115], dtype=np.uint8)
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


def get_ROI_hole(mask):
    himg, wimg = mask.shape[:2]
    ROI = []
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    for cnt in contours:
        if cv.contourArea(cnt) < 100:
            continue
        have_hole = False
        x, y, w, h = cv.boundingRect(cnt)
        mask_crop = mask[y:y+h, x:x+w]
        not_mask_crop = cv.bitwise_not(mask_crop)
        hole_contours = cv.findContours(
            not_mask_crop, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        # print len(hole_contours)
        for hole_cnt in hole_contours:
            # print cv.contourArea(hole_cnt)
            if cv.contourArea(hole_cnt) < 100 :
                continue
            hole_himg,hole_wimg = not_mask_crop.shape[:2]
            x_hole,y_hole,w_hole,h_hole = cv.boundingRect(hole_cnt)
            top_excess = (y_hole < 0.05*hole_himg)
            # bot_excess = ((y_hole+h_hole) > (0.95*hole_himg))
            bot_excess = False
            right_excess = ((x_hole+w_hole) > (0.95*hole_wimg))
            left_excess = (x_hole < (0.05*hole_wimg))
            window_excess = top_excess or bot_excess or right_excess or left_excess
            print top_excess , bot_excess , right_excess , left_excess
            print float(w_hole*h_hole)/float(w*h)
            print not (window_excess)
            if not(window_excess) and float(w_hole*h_hole)/float(w*h) > 0.2:
                have_hole = True
        mask_area = cv.countNonZero(mask_crop)
        not_mask_area = cv.countNonZero(not_mask_crop)
        percent_not_mask = float(not_mask_area) / \
            float(mask_area + not_mask_area)
        top_excess = (y < 0.05*himg)
        bot_excess = ((y+h) > 0.95*himg)
        right_excess = ((x+w) > 0.95*wimg)
        left_excess = (x < 0.05*wimg)
        window_excess = top_excess or bot_excess or right_excess or left_excess
        if not window_excess:
        #and (percent_not_mask < 0.65 and percent_not_mask > 0.35) and have_hole :
            ROI.append(cnt)
            print percent_not_mask
    return ROI


def find_red_hole(size):
    global img_top, img_top_res
    while img_top is None and not rospy.is_shutdown():
        img_is_none()
    mask = get_object(img=img_top, color="red")
    ROI = get_ROI_hole(mask)

    ROI = sorted(ROI, key=cv.contourArea)

    if ROI == []:
        mode = 1
        print_result("NOT FOUND", color_text.RED)
    elif len(ROI) == 1:
        mode = 2
        hole = ROI[0]
        print_result("FOUND ONE HOLE", color_text.GREEN)
    elif len(ROI) == 2:
        mode = 3
        if size == 'big':
            hole = ROI[0]
        elif size == 'small':
            hole = ROI[1]
        else:
            print_result("ERROR SIZE", color_text.RED_HL)
        print_result("FOUND TWO HOLE", color_text.GREEN+color_text.UNDERLINE)
    elif len(ROI) > 2:
        mode = 4
        if size == 'big':
            hole = ROI[-1]
        elif size == 'small':
            hole = ROI[-2]
        else:
            print_result("ERROR SIZE", color_text.RED_HL)
        print_result("FOUND BUT HAVE SOME NOISE", color_text.YELLOW)

    if mode == 1:
        publish_result(img_top_res, 'bgr', pub_topic + 'hole/red/result')
        publish_result(mask, 'gray', pub_topic + 'hole/red/mask')
        return message()
    elif mode == 2 or mode == 3 or mode == 4:
        himg, wimg = img_top.shape[:2]
        x, y, w, h = cv.boundingRect(hole)
        cv.rectangle(img_top_res, (x, y), (x+w, y+h), (0, 255, 0), 5)
        w_h_ratio = 1.0*w/h
        print w_h_ratio
        if w_h_ratio < 0.93:
            print_result('ASSUME AS '+color_text.GREEN_HL +
                         'SMALL'+color_text.DEFAULT+' HOLE')
        else:
            print_result('ASSUME AS '+color_text.YELLOW_HL +
                         'BIG'+color_text.DEFAULT+'HOLE')
        cx = int(x + (w/2))
        cy = int(y + (h/2))
        area = float(w*h)/float(wimg*himg)
        cv.circle(img_top_res, (cx, cy), 5, (0, 0, 255), 1)
        cv.circle(img_top_res, (cx, cy), 10, (0, 0, 255), 1)
        cv.line(img_top_res, (cx-15, cy), (cx+15, cy), (0, 0, 255), 1)
        cv.line(img_top_res, (cx, cy-15), (cx, cy+15), (0, 0, 255), 1)
        cx = Aconvert(cx, wimg)
        cy = -1.0*Aconvert(cy, himg)
        publish_result(img_top_res, 'bgr', pub_topic + 'hole/red/result')
        publish_result(mask, 'gray', pub_topic + 'hole/red/mask')
        return message(cx=cx, cy=cy, area=area, appear=True, mode=mode)


def find_yellow_hole():
    global img_top, img_top_res
    while img_top is None and not rospy.is_shutdown():
        img_is_none()
    mask = get_object(img=img_top, color="yellow")
    ROI = get_ROI_hole(mask)
    if ROI == []:
        mode = 1
        print_result("NOT FOUND", color_text.RED)
    elif len(ROI) == 1:
        mode = 2
        hole = ROI[0]
        print_result("FOUND ONE HOLE", color_text.GREEN)
    elif len(ROI) > 1:
        mode = 3
        hole = max(ROI, key=cv.contourArea)
        print_result("FOUND BUT HAVE SOME NOISE", color_text.YELLOW)

    if mode == 1:
        publish_result(img_top_res, 'bgr', pub_topic + 'hole/yellow/result')
        publish_result(mask, 'gray', pub_topic + 'hole/yellow/mask')
        return message()
    elif mode == 2 or mode == 3:
        himg, wimg = img_top.shape[:2]
        x, y, w, h = cv.boundingRect(hole)
        cv.rectangle(img_top_res, (x, y), (x+w, y+h), (0, 255, 0), 5)
        cx = int(x + (w/2))
        cy = int(y + (h/2))
        area = float(w*h)/float(wimg*himg)
        cv.circle(img_top_res, (cx, cy), 5, (0, 0, 255), 1)
        cv.circle(img_top_res, (cx, cy), 10, (0, 0, 255), 1)
        cv.line(img_top_res, (cx-15, cy), (cx+15, cy), (0, 0, 255), 1)
        cv.line(img_top_res, (cx, cy-15), (cx, cy+15), (0, 0, 255), 1)
        cx = Aconvert(cx, wimg)
        cy = -1.0*Aconvert(cy, himg)
        publish_result(img_top_res, 'bgr', pub_topic + 'hole/yellow/result')
        publish_result(mask, 'gray', pub_topic + 'hole/yellow/mask')
        return message(cx=cx, cy=cy, area=area, appear=True, mode=mode)


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
        window_excess = False
        w_h_ratio = 1.0*w/h
        print mask_area,not_mask_area
        #if not window_excess and percent_mask > 0.5:
        ROI.append(cnt)
    return ROI


def find_handle():
    global img_top, img_top_res
    while img_top is None and not rospy.is_shutdown():
        img_is_none()
    himg, wimg = img_top.shape[:2]
    mask = get_object(img=img_top, color="yellow")
    ROI = get_ROI_handle(mask)
    if ROI == []:
        mode = 1
        print_result("NOT FOUND", color_text.RED)
    elif len(ROI) == 1:
        mode = 2
        handle = ROI[0]
        print_result("FOUND HANDLE", color_text.GREEN)
    elif len(ROI) > 1:
        mode = 3
        handle = max(ROI, key=cv.contourArea)
        print_result("FOUND BUT HAVE SOME NOISE", color_text.YELLOW)

    if mode == 1:
        publish_result(img_top_res, 'bgr', pub_topic + 'handle/result')
        publish_result(mask, 'gray', pub_topic + 'handle/mask')
        return message()
    elif mode == 2 or mode == 3:
        x, y, w, h = cv.boundingRect(handle)
        cv.rectangle(img_top_res, (x, y), (x+w, y+h), (0, 255, 0), 5)
        area = 1.0*w*h/(wimg*himg)
        cx = x+(w/2)
        cy = y+(h/2)
        cv.circle(img_top_res, (cx, cy), 3, (0, 0, 255), -1)
        cx = Aconvert(cx, wimg)
        cy = -1.0*Aconvert(cy, himg)
        publish_result(img_top_res, 'bgr', pub_topic + 'handle/result')
        publish_result(mask, 'gray', pub_topic + 'handle/mask')
        return message(cx=cx, cy=cy, area=area, appear=True, mode=mode)


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
