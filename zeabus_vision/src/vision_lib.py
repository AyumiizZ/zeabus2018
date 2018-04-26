#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from zeabus_vision_task.msg import *
from zeabus_vision_task.srv import *
from cv_bridge import CvBridge


def print_result(msg):
    """
        print ('<----------') + str(msg) + ('---------->')
    """
    print ('<----------') + str(msg) + ('---------->')


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

def get_topic(cam,world):
    topic = None
    if cam == "front":
        if world == "real":
            topic = "/top/center/image_raw/compressed"
        if world == "sim":
            topic = "/syrena/front_cam/image_raw/compressed"
    if cam == "bottom":
        if world == "real":
            topic = "/bottom/left/image_raw/compressed"
        if world == "sim":
            topic = "/syrena/bottom_cam/image_raw/compressed"
    return topic

# def range_str2array(string):
#     string = string.split(',')
#     return np.array([int(string[0]), int(string[1]), int(string[2])], dtype=np.uint8)


# def get_color(color, time, mission):
#     lower = None
#     upper = None
#     color_list = ['orange', 'white', 'yellow', 'red', 'black', 'green']
#     if color in color_list:
#         lower = rospy.get_param(
#             '/color_range_' + str(mission) + '/color_' + str(time) + '/lower_' + str(color))
#         upper = rospy.get_param(
#             '/color_range_' + str(mission) + '/color_' + str(time) + '/upper_' + str(color))
#         lower = range_str2array(lower)
#         upper = range_str2array(upper)
#         print "FOUND"
#     print(lower, upper)
#     return lower, upper

def convert(inp, full):
    """
        Convert cx cy to int in range of -1 to 1
        Returns:
            float: result
    """
    inp = float(inp)
    full = float(full)
    res = (inp - (full/2.0))/(full/2.0)
    return res
