#!/usr/bin/python2.7
import rospy
import matplotlib as plt
import cv2 as cv
from zeabus_vision.msg import vision_roulette
from zeabus_vision.srv import vision_srv_roulette
from cv_bridge import CvBridge, CvBridgeError
from vision_lib import *
from sensor_msgs.msg import CompressedImage, Image

img = None
img_res = None
sub_sampling = 1
pub_topic = "/vision/roulette/"

def mission_callback(msg):
    print_result('mission_callback')

    task = msg.task.data
    req = msg.req.data
    color = ['red','green','black','yellow']
    print('task: ', str (task),'req: ',str(req))
    if task == 'roulette' and req in color:
        return find_roulette(req)

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    img_res = img.copy()

def message(cx1=-1, cx2=-1, cy1=-1, cy2=-1,appear = False):
    m = vision_roulette()
    m.cx1 = cx1
    m.cx2 = cx2
    m.cy1 = cy1
    m.cy2 = cy2
    m.appear = appear
    print(m)
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
        lower = np.array([0, 120, 0], dtype=np.uint8)
        upper = np.array([37, 255, 255], dtype=np.uint8)
    elif color == 'green' :
        lower = np.array([0, 120, 0], dtype=np.uint8)
        upper = np.array([37, 255, 255], dtype=np.uint8)
    elif color == 'black' :
        lower = np.array([0, 120, 0], dtype=np.uint8)
        upper = np.array([37, 255, 255], dtype=np.uint8)
    elif color == 'yellow' :
        lower = np.array([20, 120, 0], dtype=np.uint8)
        upper = np.array([62, 255, 255], dtype=np.uint8)
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
    sum_area = 0
    count = 0
    cnt = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    if len(cnt) == 1:
            # cnt = max(cnt, key=cv.contourArea)
        appear = True
        this_area = cv.contourArea(cnt[0])
            # sim
        if this_area > 2000:
            # real world
            # if this_area > 4000:
            M = cv.moments(cnt[0])
            ROI_cx1 = int(M["m10"]/M["m00"])
            ROI_cy1 = int(M['m01']/M['m00']) 
            if ROI_cx1 >= 0.05 * wimg and ROI_cx1 <= 0.95 * wimg:
                cv.circle(img_res, (ROI_cx1, ROI_cy1), 10, (0, 0, 255), -1)
                cx1.append(ROI_cx1)
                cy1.append(ROI_cy1)
                sum_area += this_area
                count += 1
        # avg_area = 0 if count == 0 else sum_area/count
        cx1 = cx1[::-1]
        cy1 = cy1[::-1]
        return cx1, cy1 ,cx2 ,cy2 ,appear
    elif len(cnt) == 2 :
        appear = True
        area_1 = cv.contourArea(cnt[0])
        area_2 = cv.contourArea(cnt[1])
        if area_1 > 2000 and area_2 >2000 :
            M_1= cv.moments(cnt[0])
            ROI_cx1 = int(M_1["m10"]/M_1["m00"])
            ROI_cy2 = int(M_1['m01']/M_1['m00']) 
            if ROI_cx1 >= 0.05 * wimg and ROI_cx1 <= 0.95 * wimg:
                cv.circle(img_res, (ROI_cx1, ROI_cy1), 10, (0, 0, 255), -1)
                cx.append(ROI_cx1)
                cy.append(ROI_cy1)
                sum_area += area_1
                count += 1
        # avg_area = 0 if count == 0 else sum_area/count
                cx1 = cx1[::-1]
                cy1 = cy1[::-1]
                M_2 = cv.moments(cnt[1])
                ROI_cx2 = int(M_2["m10"]/M_2["m00"])
                ROI_cy2 = int(M_2['m01']/M_2['m00']) 
                if ROI_cx2 >= 0.05 * wimg and ROI_cx2 <= 0.95 * wimg:
                    cv.circle(img_res, (ROI_cx2, ROI_cy2), 10, (0, 0, 255), -1)
                    cx2.append(ROI_cx2)
                    cy2.append(ROI_cy2)
                    sum_area += area_2
                    count += 1
        # avg_area = 0 if count == 0 else sum_area/count
                    cx2 = cx2[::-1]
                    cy2 = cy2[::-1]
        return cx1, cy1 , cx2 , cy2 ,appear
    return cx1, cy1,cx2,cy2,appear


def find_roulette(color) :
    global img, img_res
    while img is None and not rospy.is_shutdown():
        print('img is none.\nPlease check topic name or check camera is running')
    
    mask = get_object(color)
    cx1, cy1, cx2, cy2, appear = get_cx(mask)
    if cx1 == [] and cy1 == [] and cy2 ==[] and cx2 == []:
        mode = 1
    elif len(cx1) == 1 and len(cy1) == 1 and cx2 == [] and cy2 == []:
        mode = 2
        cv.circle(img_res, (cx1[0], cy1[0]), 10, (255, 0, 0), -1)
    elif len(cx1) ==1 and len(cy1) == 1 and len(cy2) == 1 and len(cx2) == 1:
        mode = 3
        cv.circle(img_res, (cx1[0], cy1[0]), 10, (255, 0, 0), -1)
        cv.circle(img_res, (cx2[0] ,cy2[0]), 10, (255, 0, 0), -1)
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
        return message(cx1=return_cx1, cy1=return_cy1, appear=True)
    elif mode == 3:
        print_result("MODE 3: CAN FIND 2 CX AND 2 CY")
        himg, wimg = img.shape[:2]
        return_cx1 = 1.0*(cx1[0] - (wimg/2))/(1.0*wimg/2)
        return_cy1 = -1.0*(cy1[0] - (himg/2))/(1.0*himg/2)
        return_cx2 = 1.0*(cx2[0] - (wimg/2))/(1.0*wimg/2)
        return_cy2 = -1.0*(cy2[0] - (himg/2))/(1.0*himg/2)
        
        # return_area = (1.0*area*16)/(himg*wimg)
        # return_degrees = find_angle(cx, cy)
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx1=return_cx1, cy1=return_cy1, cx2 = return_cx2, cy2 = return_cy2, appear=True)


if __name__ == '__main__':
    rospy.init_node('vision_roulette', anonymous=True)
    print_result("INIT NODE")

    # sim
    # TOPIC = "/syrena/bottom_cam/image_raw/compressed"
    # real world
    TOPIC = "/bottom/left/image_raw/compressed"

    rospy.Subscriber(TOPIC, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")

    rospy.Service('vision_roulette', vision_srv_roulette(),
                  mission_callback)
    print_result("INIT SERVICE")

    rospy.spin()
    print_result("END PROGRAM")


