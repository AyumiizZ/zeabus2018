#!/usr/bin/python2.7
import rospy
import matplotlib as plt
import cv2 as cv
from zeabus_vision.msg import vision_roulette
from zeabus_vision.srv import vision_srv_roulette
from cv_bridge import CvBridge, CvBridgeError
from vision_lib import *
from sensor_msgs.msg import CompressedImage, Image
'''
    part of color range 
    import library    
'''
import sys

img = None
img_res = None
sub_sampling = 1
pub_topic = "/vision/roulette/"

'''
    part of color range 
    Declare global variable for contain param value from launch file (mission.launch)
'''
number = None
mission = None
camera_position = None

def mission_callback(msg):
    print_result('mission_callback')

    task = msg.task.data
    req = msg.req.data
    color = ['red','green','black']
    print('task: ', str (task),'req: ',str(req))
    if task == 'roulette' and req in color:
        return find_roulette(req)

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    size = 500
    r = 1.0*size / img.shape[1]
    dim = (size, int(img.shape[0] * r))
    resized = cv.resize(img, dim, interpolation = cv.INTER_AREA)
    img = resized
    img_res = img.copy()

def message(cx=-1, cy=-1,area = -1,appear = False):
    m = vision_roulette()
    m.cx = cx
    m.cy = cy
    m.area = area
    m.appear = appear
    # print(m)
    return m

def get_object(color):
    """
        get mask from picture and remove some noise
        Returns:
            mask (ONLY PATH AREA)
    """
    global img
    

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    
    
    # sim
    if color == 'red' :
        lower = np.array([139, 0, 0], dtype=np.uint8)
        upper = np.array([255, 255, 255], dtype=np.uint8)
    elif color == 'green' :
        lower = np.array([50, 119, 4], dtype=np.uint8)
        upper = np.array([88, 255, 255], dtype=np.uint8)
    elif color == 'black' :
        lower = np.array([83, 35, 13], dtype=np.uint8)
        upper = np.array([142, 179, 43], dtype=np.uint8)
    # elif color == 'yellow' :
    #     lower = np.array([20, 120, 0], dtype=np.uint8)
    #     upper = np.array([62, 255, 255], dtype=np.uint8)
    # # real world
    # lower = np.array([20, 120, 0], dtype=np.uint8)
    # upper = np.array([62, 255, 255], dtype=np.uint8)

    # lower,upper = get_color('yellow','morning','path')
    mask = cv.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv.GaussianBlur(mask, (5, 5), 0)
    mask = cv.erode(mask, kernel)
    mask = cv.erode(mask, kernel)
    mask = cv.dilate(mask, kernel)
    mask = cv.dilate(mask, kernel)
    return mask

def get_cx(mask):
    global img
    himg, wimg = img.shape[:2]
    cx1 = []
    cx2 = []
    cy1 = []
    cy2 = []
    appear = False
    # sum_area = 
    count = 0
    cnt = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    if len(cnt) >= 2 :
        cnt = sorted(cnt,key=cv.contourArea)[:2]
        # cv.imshow('cnt',bo2)
    # cv.imshow('cnt',hsv)
    if len(cnt) == 1 :
        # appear = True
        area = cv.contourArea(cnt[0])
        if area > 5000:
            M = cv.moments(cnt[0])
            ROI_cx1 = int(M["m10"]/M["m00"])
            ROI_cy1 = int(M['m01']/M['m00']) 
            if ROI_cx1 >= 0.05 * wimg and ROI_cx1 <= 0.95 * wimg:
                cv.circle(img_res, (ROI_cx1, ROI_cy1), 10, (0, 0, 255), -1)
                cx1.append(ROI_cx1)
                cy1.append(ROI_cy1)
                appear = True
                # sum_area += tarea
                # count += 1
        # avg_area = 0 if count == 0 else sum_area/count
        area = area/(wimg*himg)
        cx1 = cx1[::-1]
        cy1 = cy1[::-1]
        return cx1, cy1 , area ,appear
    elif len(cnt) == 2 :    
        # appear = True
        area_1 = cv.contourArea(cnt[0])
        area_2 = cv.contourArea(cnt[1])
        if area_1 > 5000 and area_2 > 5000 :
            M_1= cv.moments(cnt[0])
            ROI_cx1 = int(M_1["m10"]/M_1["m00"])
            ROI_cy1 = int(M_1['m01']/M_1['m00']) 
            if ROI_cx1 >= 0.05 * wimg and ROI_cx1 <= 0.95 * wimg:
                # cv.circle(img_res, (ROI_cx1, ROI_cy1), 3, (0, 0, 255), -1)
                cx1.append(ROI_cx1)
                cy1.append(ROI_cy1)
                area = (area_1+area_2)/(wimg*himg)
                # sum_area += area_1
                # count += 1
        # avg_area = 0 if count == 0 else sum_area/count
                cx1 = cx1[::-1]
                cy1 = cy1[::-1]
                M_2 = cv.moments(cnt[1])
                ROI_cx2 = int(M_2["m10"]/M_2["m00"])
                ROI_cy2 = int(M_2['m01']/M_2['m00'])
                if abs(ROI_cx1 - (wing/2)) < abs(ROI_cx2 - (wing/2)) :
                    ROI_cx2 = ROI_cx1 
                    ROI_cy2 = ROI_cy1
                # if abs(ROI_cy1) < abs(ROI_cy2) :
                #     ROI_cy2 = ROI_cy1
                if ROI_cx2 >= 0.05 * wimg and ROI_cx2 <= 0.95 * wimg:
                    cv.circle(img_res, (ROI_cx2, ROI_cy2), 3, (0, 0, 255), -1)
                    cx2.append(ROI_cx2)
                    cy2.append(ROI_cy2)
                    # sum_area += area_2
                    # count += 1
        # avg_area = 0 if count == 0 else sum_area/count
                    cx2 = cx2[::-1]
                    cy2 = cy2[::-1]
                return cx2,cy2,area,appear
    return cx2,cy2,area,appear


def find_roulette(color) :
    global img, img_res
    '''
        part of color range 
        Use global variable    
    '''
    global mission, number, camera_position


    '''
        part of color range         
        Get color by get_color_range method in vision_lib.py
    '''
    lower, upper = get_color_range('red',camera_position, number, mission)
    print(lower,upper)
    exit(0)
    while img is None and not rospy.is_shutdown():
        print('img is none.\nPlease check topic name or check camera is running')
    
    mask = get_object(color)
    cx1, cy1, area, appear = get_cx(mask)
    if cx1 == [] and cy1 == [] :
        mode = 1
    elif len(cx1) == 1 and len(cy1) == 1 :
        mode = 2
        cv.circle(img_res, (cx1[0], cy1[0]), 10, (255, 0, 0), -1)
    if mode == 1:
        print_result("MODE 1: CANNOT FIND ROULETTE")
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message()
    elif mode == 2:
        print_result("MODE 2: CAN FIND 1 CX AND 1 CY")
        himg, wimg = img.shape[:2]
        return_cx1 = 1.0*(cx1[0] - (wimg/2))/(1.0*wimg/2)
        return_cy1 = -1.0*(cy1[0] - (himg/2))/(1.0*himg/2)
        # return_area = (1.0*area*16)/(himg*wimg)
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx1=return_cx1, cy1=return_cy1, area = area ,appear=True)

if __name__ == '__main__':
    rospy.init_node('vision_roulette', anonymous=True)
    print_result("INIT NODE")


    '''
        part of color range
        Get param when run by launch file (mission.launch)
    '''
    namespace = sys.argv[1]
    #############################################################################################
    camera_position = rospy.get_param(namespace+'/camera_position', 'null')
    mission = rospy.get_param(namespace+'/mission', 'null')
    number = rospy.get_param(namespace+'/number', 'null')
    #############################################################################################


    # sim
    # TOPIC = "/syrena/bottom_cam/image_raw/compressed"
    # real world
    TOPIC = "/bottom/left/image_raw/compressed"

    rospy.Subscriber(TOPIC, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")


    '''
        part of color range
        comment for testing while not service
    '''
    # rospy.Service('vision_roulette', vision_srv_roulette(),
    #               mission_callback)
    # print_result("INIT SERVICE")

    # rospy.spin()
    # print_result("END PROGRAM")

    while not rospy.is_shutdown():
        find_roulette('red')
