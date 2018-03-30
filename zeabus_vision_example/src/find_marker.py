#!/usr/bin/python2.7
import cv2 as cv
import rospy
import numpy as np
#from remove_noise import *
from sensor_msgs.msg import CompressedImage , Image
from zeabus_example.msg import qualification_marker
from zeabus_example.srv import robosub_marker
from cv_bridge import CvBridge , CvBridgeError
from qualify import *
img = None
img_res = None
sub_sampling = 1

def mission_callback(msg):
    print_result('mission_callback')
    req = msg.req.data
    task = msg.task.data

    print('task:', str(task), 'request:', str(req))
    if task == 'marker' :
        return find_marker() 

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                     fx=sub_sampling, fy=sub_sampling)
    img_res = img.copy()


def find_marker():
    global img,img_res
    appear = False
    pos = 99
    cx_left = -1
    cx_right = -1
    area = 0
    top_excess = False
    bot_excess = False
    left_excess = False
    right_excess = False
    while img is None and not rospy.is_shutdown() :
        print('img is none.\nPlease check topic name or check camera is running')
        break
    # img = frame.copy()
    # cv.imshow('img',img)
    img = cv.resize(img,(640,480))
    himg , wimg = img.shape[:2]
    print himg
    print wimg
    # img_res = process_gate(img_res)
    # hsv = cv.cvtColor(img_res,cv.COLOR_BGR2HSV)
    # l = np.array([8,111,109])
    # h = np.array([19,216,247])
    # mask = cv.inRange(hsv,l,h)
    mask = process_gate(img_res)
    contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    for cnt in contours:
        cnt_area = cv.contourArea(cnt)
        if cnt_area > 500 :
            appear = True
            x, y, w, h = cv.boundingRect(cnt)
            # cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), 2)
            cx_left = x
            cx_right = x+w
            img = cv.line(img,(cx_left,0),(cx_left,himg),(255,0,0),1)
            cv.putText(img, "left", (x+5,himg-30), cv.FONT_HERSHEY_TRIPLEX, 1,
                   [0, 0, 0])
            img = cv.line(img,(cx_right,0),(cx_right,himg),(255,0,0),1)
            cv.putText(img, "right", (x+w+5,himg-30), cv.FONT_HERSHEY_TRIPLEX, 1,
                   [0, 0, 0])
    print "appear = " + str(appear)
    publish_result(img,'bgr','/img_marker')
    publish_result(mask,'gray','mask')
    return message_marker(cx_left,cx_right,area,appear)

def main():
    rospy.init_node('vision_gate', anonymous=True)
    image_topic = "/syrena/front_cam/image_raw/compressed"
    # image_topic = "/top/center/image_raw/compressed"
    res_topic = "/top/center/res/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    #abc = rospy.Publisher(res_topic, CompressedImage, queue_size=10)
    print "fuck"
    rospy.Service('vision_gate', robosub_marker(), mission_callback)
    print "fuck"
    rospy.spin()



if __name__ == '__main__':
    main()
