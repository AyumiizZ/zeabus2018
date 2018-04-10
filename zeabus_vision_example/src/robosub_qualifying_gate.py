#!/usr/bin/python2.7
import cv2 as cv
import rospy
import numpy as np
#from remove_noise import *
from sensor_msgs.msg import CompressedImage , Image
from zeabus_example.msg import robosub_qualifying_gate_msg
from zeabus_example.srv import robosub_qualifying_gate_srv
from cv_bridge import CvBridge , CvBridgeError
from robosub_qualifying_lib import *
img = None
img_res = None
sub_sampling = 1
    

def mission_callback(msg):
    print_result('mission_callback')

    req = msg.req.data
    task = msg.task.data

    print('task:', str(task), 'request:', str(req))
    if task == 'gate':
        return find_gate()
    

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                     fx=sub_sampling, fy=sub_sampling)
    # img = cv.cvtColor(img,cv.COLOR_BayerBG2BGR)
    img_res = img.copy()

def find_gate () :
    global img,img_res
    appear = False
    pos = 99
    cx = -1
    area = 0
    top_excess = False
    bot_excess = False
    left_excess = False
    right_excess = False
    while img is None and not rospy.is_shutdown() :
        print('img is none.\nPlease check topic name or check camera is running')
        break
    himg , wimg = img.shape[:2]
    # mask = process_gate(img_res)
    hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    img = cv.medianBlur(img,5)
    mask = get_object(img)
    contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    c = 0
    ROI = []
    ROI_area = []
    for cnt in contours:
        cnt_area = cv.contourArea(cnt)
        if cnt_area > 5000 :
            c += 1
            appear = True
            x,y,w,h = cv.boundingRect(cnt)
            if(x < 0.05*wimg):
                left_excess = True
            if((x+w) > 0.95*wimg):
                right_excess = True
            if(y < 0.05*himg):
                top_excess = True
            if((y+h) > 0.95*himg):
                bot_excess = True
            img = cv.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
            ROI.append(cnt)
    print len(ROI)
    if len(ROI) >= 2:
        pos = 0
        temp = []
        for cnt in ROI:
            M = cv.moments(cnt)
            cx = int(M["m10"]/M["m00"])
            temp.append(cx)
        cx = sum(temp)/len(temp)
        img = cv.line(img,(cx,0),(cx,himg),(255,0,0),5)
    if len(ROI) == 2:
        pos = 0
        M = cv.moments(ROI[0])
        cx_0 = int(M["m10"] / M["m00"])
        M = cv.moments(ROI[1])
        cx_1 = int(M["m10"] / M["m00"])
        cx = int((cx_0+cx_1)/2)
        img = cv.line(img,(cx,0),(cx,himg),(255,0,0),5)
    if len(ROI) == 1:
        area = (h*w)/(himg*wimg)
        if left_excess is False and right_excess is True:
            pos = -1
            print "-1 = left"
        elif left_excess is True and right_excess is False:
            pos = 1
            print "1 = right"
        elif left_excess is True and right_excess is True:
            pos = 0
            cx = wimg/2
            img = cv.line(img,(cx,0),(cx,himg),(255,0,0),5)
        elif h < 5*w:
            pos = 0
            cx = (2*x+w)/2
            img = cv.line(img,(cx,0),(cx,himg),(255,0,0),5)
        else:
            pos = -99
    publish_result(img,'bgr','/qualify_gate/img')
    publish_result(mask,'gray','/qualify_gate/mask')
    return message(cx,pos,area,appear)


def main():
    rospy.init_node('vision_gate', anonymous=True)
    # image_topic = "/syrena/front_cam/image_raw/compressed"
    image_topic = "/top/center/image_raw/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print "init_pub_sub"
    rospy.Service('vision_gate', robosub_qualifying_gate_srv (), mission_callback)
    print "init_ser"
    rospy.spin()



if __name__ == '__main__':
    main()