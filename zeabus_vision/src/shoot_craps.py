#!/usr/bin/python2.7

import math
import rospy
import cv2 as cv
import numpy as np
from vision_lib import *
from shoot_craps.dice_detection import *
from zeabus_vision.msg import vision_dice
from sensor_msgs.msg import CompressedImage
from zeabus_vision.srv import vision_srv_dice

img = None
img_res = None
sub_sampling = 0.5
image_topic = '/top/center/image_raw/compressed'
publish_topic = '/vision/shoot_craps/'


def mission_callback(msg):
    print_result('mission_callback')
    task = msg.task.data
    print('task:', str(task))
    if task == 'dice':
        return find_dice()


def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    img_res = img.copy()


def message(appear=False, point=1, cx=-1, cy=-1):
    m = vision_dice()
    m.appear = appear
    m.point = point
    m.cx = cx
    m.cy = cy
    print(m)
    return m



def find_dice():
    global img, publish_topic

    while not rospy.is_shutdown():
        if img is None:
            continue
        cl = clahe(img)
        # gray = cv.cvtColor(cl, cv.COLOR_BGR2GRAY)
        # equ = equalization_gray(gray)
        img_result = run(img)
        # get_rectangle(cl)
        # publish_result(gray, 'gray', publish_topic + 'gray')
        # publish_result(equ, 'gray', publish_topic + 'equ')
        # publish_result(cl, 'bgr', publish_topic + 'clahe')
        publish_result(img_result, 'bgr', publish_topic + 'result')

        # cv.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('vision_dice', anonymous=False)
    print_result("INIT NODE")

    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")

    # rospy.Service('vision_path', vision_srv_dice(),
    #               mission_callback)
    # print_result("INIT SERVICE")

    # rospy.spin()
    # print_result("END PROGRAM")
    find_dice()
