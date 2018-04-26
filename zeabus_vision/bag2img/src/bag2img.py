#!/usr/bin/env python

import cv2
import rosbag
import rospy
import numpy as np
import math
import rosbag
import argparse
from sensor_msgs.msg import CompressedImage


class Bag2Img:

    def __init__(self, filepath, topic_left, topic_right):
        rospy.init_node('Bag2Images', anonymous=False)
        self.topic_left = '/' + topic_left + '/image_raw/compressed'
        self.topic_right = '/' + topic_right + '/image_raw/compressed'
        self.bag = rosbag.Bag(filepath)
        self.count = 0
        self.img_left = None
        self.img_right = None

    def camera_left_callback(self, msg):
        arr = np.fromstring(msg.data, np.uint8)
        self.img = cv2.imdecode(arr, 1)

    def camera_right_callback(self, msg):
        arr = np.fromstring(msg.data, np.uint8)
        self.img = cv2.imdecode(arr, 1)

    def capture(self):
        while not rospy.is_shutdown():
            key = cv2.waitKey(1) & 0xff
            name_l = 'left-' + str(self.count) + '.jpg'
            name_r = 'right-' + str(self.count) + '.jpg'
            cv2.imwrite(name_l, self.img_l)
            cv2.imwrite(name_r, self.img_r)
            self.count += 1
            if key == ord('q'):
                break
            rospy.sleep(1)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rate = rospy.Rate()
    bag2jpg = Bag2Img('calibrate.bag')
    bag2jpg.capture()
