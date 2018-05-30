#!/usr/bin/python2.7
import roslib
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import numpy as np


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.topic_r = '/my_stereo/right/image_raw'
        self.topic_l = '/my_stereo/left/image_raw'
        self.image_sub_r = rospy.Subscriber(
            self.topic_r, Image, self.callback_r)
        self.image_sub_l = rospy.Subscriber(
            self.topic_l, Image, self.callback_l)
        self.image_r = None
        self.image_l = None
        self.count = 0
        self.path = rospkg.RosPack().get_path('zeabus_vision')

    def callback_r(self, data):
        try:
            self.image_r = self.bridge.imgmsg_to_cv2(data, "mono8")
            # self.image_r = cv.resize(self.image_r, (0, 0), fx=0.5, fy=0.5)
        except CvBridgeError as e:
            print(e)

    def callback_l(self, data):
        try:
            self.image_l = self.bridge.imgmsg_to_cv2(data, "mono8")
            # self.image_l = cv.resize(self.image_l, (0, 0), fx=0.5, fy=0.5)
        except CvBridgeError as e:
            print(e)

    def capture(self):
        self.image_l_save = self.path + \
            '/images/left/left' + ("%03d") % self.count + '.jpg'
        self.image_r_save = self.path + \
            '/images/right/right' + ("%03d") % self.count + '.jpg'
        print 'Capture images: [',self.count,']'
        cv.imwrite(self.image_l_save, self.image_l)
        cv.imwrite(self.image_r_save, self.image_r)
        self.count += 1

    def callback_btn(self):
        pass

    def run(self):
        # termination criteria

        while not rospy.is_shutdown():
            if self.image_l is None or self.image_r is None:
                continue

            # Find the chess board corners
            ret_r, corners_l = cv.findChessboardCorners(
                self.image_l, (8, 7), None)
            ret_l, corners_r = cv.findChessboardCorners(
                self.image_r, (8, 7), None)
            img_l = cv.cvtColor(self.image_l.copy(), cv.COLOR_GRAY2BGR)
            img_r = cv.cvtColor(self.image_r.copy(), cv.COLOR_GRAY2BGR)

            if ret_r == True and ret_r == True:
                cv.drawChessboardCorners(img_l, (8, 7), corners_l, ret_l)
                cv.drawChessboardCorners(img_r, (8, 7), corners_r, ret_r)
            img_l = cv.resize(img_l, (0, 0), fx=0.4, fy=0.4)
            img_r = cv.resize(img_r, (0, 0), fx=0.4, fy=0.4)
            cv.imshow('left', img_l)
            cv.imshow('right', img_r)
            k = cv.waitKey(2) & 0xff
            if k == ord('c'):
                self.capture()
            elif k == ord('q'):
                break


def main():
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    ic.run()


if __name__ == '__main__':
    main()
