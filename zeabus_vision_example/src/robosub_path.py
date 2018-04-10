#!/usr/bin/python2.7
import rospy
import cv2 as cv
import numpy as np
from zeabus_example.msg import robosub_path_msg
from zeabus_example.srv import robosub_path_srv
from sensor_msgs.msg import CompressedImage , Image
from cv_bridge import CvBridge , CvBridgeError
img = None
img_res = None

def print_result (msg) :
    print ('<-----------') + str(msg) + ('---------->')

def get_object (img) :
    hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    lower_yellow = np.array([19,125,68])
    upper_yellow = no.array([32,244,247])
    obj = cv.inRange(hsv,lower_yellow,upper_yellow)
    return obj

def remove_noise (mask) :
    kernel = get_kernel('rect',(5,9))
    erode = cv.erode(mask,kernel)

    kernel = get_kernel('rect',(5,15))
    dilate = cv.dilate(erode,kernel)

    kernel = get_kernel('rect',(2,19))
    erode = cv.erode(dilate,kernel)

    return erode

def get_kernel(shape = 'rect',ksize =(5,5)) :
    if shape == 'rect' :
        return cv.getStructuringElement(cv.MORPH_RECT,ksize)

def process_gate(frame) :
    mask_change_color = prepocessing(frame)
    obj = get_object(mask_change_color)
    remove = remove_noise(obj)
    return remove

def mission_callback(msg):
    print_result('mission_callback')

    task = msg.task.data

    print('task:', str(task))
    if task == 'path' :
        return find_path()

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

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                     fx=sub_sampling, fy=sub_sampling)
    img_res = img.copy()

def message(cx=0,cy=0,degree=0,appear=False):
    global c_img
    print(appear,cx,cy,degree)
    m = robosub_path_msg()
    m.appear = appear
    m.cx = x
    m.cy = y
    m.degree = degree
    print(m)
    return m

def img_callback(msg):
    global img

    arr = np.fromstring( msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (640, 512))

def img_bot_callback(msg):
    global img_bot, height_bot, width_bot
    arr = np.fromstring( msg.data, np.uint8)
    img_bot = cv.resize(cv.imdecode(arr, 1), (640, 512))
    height_bot, width_bot,_ = img_bot.shape

def main():
    rospy.init_node('vision_path', anonymous=True)
    image_topic = "/syrena/front_cam/image_raw/compressed"
    # image_topic = "/top/center/image_raw/compressed"
    res_topic = "/top/center/res/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    #abc = rospy.Publisher(res_topic, CompressedImage, queue_size=10)
    print "init_pub_sub"
    rospy.Service('vision_path', robosub_path_srv (), mission_callback)
    print "init_ser"
    rospy.spin()

if __name__ == '__main__':
    main()