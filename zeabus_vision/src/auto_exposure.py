#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
import math
import dynamic_reconfigure.client
import constant as CONST
from vision_lib import *



class AutoExposure:


    def __init__(self, subTopic, clientName, EVdefault=1, EVmin=0.5, camera_position='front'):
        print_result("init_node_auto_exposure")
        self.hsv = None
        self.image = None
        self.subTopic = subTopic
        self.clientName = clientName
        self.EVdefault = EVdefault
        self.minEV = EVmin
        self.subsampling = 0.5
        self.subImage = rospy.Subscriber(
            subTopic, CompressedImage, self.img_callback,  queue_size=10)
        self.client = dynamic_reconfigure.client.Client(self.clientName)
        print_result('set_client')
        self.set_param('exposure', self.EVdefault)

    def get_mode(self,data):
        if len(data.shape) > 1:
            data = data.ravel()
        count = np.bincount(data)
        max = count.max()
        count = list(count)
        return count.index(max)

    
    def img_callback(self, msg):
        arr = np.fromstring(msg.data, np.uint8)
        self.image = cv2.resize(cv2.imdecode(
            arr, 1), (0,0),fx=self.subsampling,fy=self.subsampling)
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

    def set_param(self, param, value):
        value = max(self.minEV, value)
        params = {str(param): value}
        config = self.client.update_configuration(params)


    def get_param(self, param):
        value = rospy.get_param("/" + str(self.clientName) + str(param), None)
        return value

    def adjust_exposure_time(self):
        while not rospy.is_shutdown():
            if self.hsv is None:
                print_result('image is none')
                continue

          
            h, s, v = cv2.split(self.hsv)
            vOneD = v.ravel()
            vMode = self.get_mode(vOneD)
       
            ev = self.get_param('exposure')
            print_result('Exposure')
            print_result(ev)
            print_result('V mode')
            print_result(vMode)
            if ev is None:
                continue
            if vMode >= 200:
                ev -= 0.04
                self.set_param('exposure', ev)
            elif vMode <= 50:
                ev += 0.04
                self.set_param('exposure', ev)
            rospy.sleep(0.1)

    def inrange_ratio(self, min, ratio, max):
        if min <= ratio <= max:
            return True
        return False

    def trimmed(self, data, trimmedValue):
        data = list(data)
        for i in range(0, trimmedValue[0]):
            data = filter(lambda a: a != i, data)
        for i in range(trimmedValue[1], 256):
            data = filter(lambda a: a != i, data)
        return data
