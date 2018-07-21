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
world = "real"

def mission_callback(msg):
    print_result('mission_callback',color_text.CYAN)

    task = msg.task.data
    req = msg.req.data
    color = ['red','green','black']
    print('task: ', str (task),'req: ',str(req))
    if task == 'roulette' and req == 'find':
        return find_roulette()

def image_callback(msg):
    global img, sub_sampling, img_res
    arr = np.fromstring(msg.data, np.uint8)
    img = cv.resize(cv.imdecode(arr, 1), (0, 0),
                    fx=sub_sampling, fy=sub_sampling)
    himg, wimg = img.shape[:2]
    img = cv.resize(img, (int(wimg/3), int(himg/3)))
    img_res = img.copy()

def message(cx=-1, cy=-1,area = -1,appear = False):
    m = vision_roulette()
    m.cx = cx
    m.cy = cy
    m.area = area
    m.appear = appear
    print(m)
    return m

def get_object():
    global img
    b,g,r = cv.split(img)
    hsv_map = cv.applyColorMap(b,cv.COLORMAP_HSV)
    hsv = cv.cvtColor(hsv_map, cv.COLOR_BGR2HSV)
    lower = np.array([0, 0, 0], dtype=np.uint8)
    upper = np.array([30, 255, 255], dtype=np.uint8)
    mask = cv.inRange(hsv, lower, upper)
    publish_result(hsv_map,'bgr',pub_topic + 'hsv_map')
    return mask


def get_ROI(mask):
    global img
    contours = cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[1]
    himg,wimg = img.shape[:2]
    ROI = []
    for cnt in contours:
        if cv.contourArea(cnt) < 100:
            continue
        x,y,w,h = cv.boundingRect(cnt)
        top_excess = (y < 0.05 *himg)
        bot_excess = (y+h > 0.95*himg)
        left_excess = (x < 0.05*wimg)
        right_excess = (x+w > 0.95*wimg)
        window_excess = top_excess or bot_excess or left_excess or right_excess
        if not window_excess:
            ROI.append(cnt)
    return ROI

def find_roulette() :
    global img, img_res
    # mode = 1
    while img is None and not rospy.is_shutdown():
        img_is_none()
    
    mask = get_object()
    ROI = get_ROI(mask)
    if ROI == [] :
        mode = 1
        print_result("MODE 1: CANNOT FIND ROULETTE",color_text.RED)
    elif len(ROI) == 1 :
        mode = 2
        roulette = ROI[0]
        print_result("MODE 2: CAN FIND ROULETTE",color_text.GREEN)
    elif len(ROI) >= 2:
        mode = 3
        roulette = max(ROI,key = cv.contourArea)
        print_result("MODE 3: CAN FIND BUT HAVE SOME NOISE",color_text.YELLOW)


    if mode == 1:
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message()
    elif mode == 2 or mode == 3:
        himg,wimg = img.shape[:2]
        x, y, w, h = cv.boundingRect(roulette)
        cv.rectangle(img_res, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cx = x+(w/2)
        cy = y+(h/2)
        cv.circle(img_res,(cx,cy),3,(0,0,255),-1)
        cx = Aconvert(cx, wimg)
        cy = -1.0*Aconvert(cy, himg)
        area = (1.0*w*h)/(wimg*himg)
        publish_result(img_res, 'bgr', pub_topic + 'img_res')
        publish_result(mask, 'gray', pub_topic + 'mask')
        return message(cx=cx, cy=cy, area = area ,appear=True)

if __name__ == '__main__':
    rospy.init_node('node_roulette', anonymous=False)
    print_result("INIT NODE")

    image_topic = get_topic("bottom",world)
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print_result("INIT SUBSCRIBER")

    rospy.Service('vision_roulette', vision_srv_roulette(),
                  mission_callback)
    print_result("INIT SERVICE")

    rospy.spin()
    print_result("END PROGRAM")


