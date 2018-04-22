#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from zeabus_vision_task.msg import *
from zeabus_vision_task.srv import *
from cv_bridge import CvBridge


def print_result(msg):
    print ('<----------') + str(msg) + ('---------->')


def publish_result(img, type, topicName):
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


def range_str2array(string):
    string = string.split(',')
    return np.array([int(string[0]), int(string[1]), int(string[2])], dtype=np.uint8)


def get_color(color, time, mission):
    lower = None
    upper = None
    color_list = ['orange', 'white', 'yellow', 'red', 'black', 'green']
    if color in color_list:
        lower = rospy.get_param(
            '/color_range_' + str(mission) + '/color_' + str(time) + '/lower_' + str(color))
        upper = rospy.get_param(
            '/color_range_' + str(mission) + '/color_' + str(time) + '/upper_' + str(color))
        lower = range_str2array(lower)
        upper = range_str2array(upper)
        print "FOUND"
    print(lower, upper)
    return lower, upper
