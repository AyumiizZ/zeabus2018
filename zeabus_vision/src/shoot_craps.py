#!/usr/bin/python2.7

import math
import rospy
import cv2 as cv
import numpy as np
from vision_lib import *
from shoot_craps.dice_detection import *
from zeabus_vision.msg import vision_shoot_craps
from sensor_msgs.msg import CompressedImage
from zeabus_vision.srv import vision_srv_shoot_craps
import constant as CONST
img = None
img_res = None
sub_sampling = 0.5
image_topic = '/stereo/right/image_raw/compressed'
# image_topic = '/top/center/image_raw/compressed'
publish_topic = '/vision/shoot_craps/'
hit_color_lenght = 50

def mission_callback(msg):
    print_result('mission_callback')
    task = msg.task.data
    print('task:', str(task))
    if task == 'shoot_craps':
        return find_dice()
    elif task == 'shoot_craps_hit':
        return check_hit()


def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    img_res = img.copy()



def message(dice_data = None):
    m = vision_shoot_craps()
    if dice_data is None:
        appear, point, cx, cy, area = False, -1, -1, -1, -1
        m.appear.append(appear)
        m.point.append(int(point))
        m.cx.append(cx)
        m.cy.append(cy)
        m.area.append(area)
        return m
    for d in dice_data.keys():
        appear, point, cx, cy, area = False, d, -1, -1, -1
        if not dice_data[d] is None:
            appear = True
            (cx,cy), _, _, _, area = dice_data[d]
        m.appear.append(appear)
        m.point.append(int(point))
        m.cx.append(cx)
        m.cy.append(cy)
        m.area.append(area)
    print(m)
    return m


def find_dice():
    global img, publish_topic

    if img is not None:    
        img_result, dice_data, mask_th = run(img)
        publish_result(img_result, 'bgr', publish_topic + 'result')
        publish_result(mask_th, 'gray', publish_topic + 'mask')
        m = message(dice_data)
        return m
    return message()

def check_hit():
    global img, publish_topic
    print('CHECK HIT')
    if img is not None:    
        m = vision_shoot_craps()
        gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        gray = set(list(gray.ravel()))
        publish_result(img, 'bgr', publish_topic + 'result')
        
        appear, point, cx, cy, area = False, -1, -1, -1, -1
        m.point.append(int(point))
        m.cx.append(cx)
        m.area.append(area)
        m.cy.append(cy)
        print(len(gray))
        if len(gray) <= hit_color_lenght:
            appear = True 
        m.appear.append(appear)
        print(m)
        return m
    return message()

if __name__ == '__main__':
    rospy.init_node('vision_dice', anonymous=False)
    print_result("INIT NODE")

    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")

    rospy.Service('vision_shoot_craps', vision_srv_shoot_craps(),mission_callback)
    print_result("INIT SERVICE")
    print_result("WAIT Request...")

    rospy.spin()
    print_result("END PROGRAM")
