#!/usr/bin/env python
import cv2 as cv
import numpy as np
import rospkg
import rospy
import dynamic_reconfigure.client
import constant as CONST
from sensor_msgs.msg import CompressedImage

img = None
img_id = 0
img_width = CONST.IMAGE_BOTTOM_WIDTH
img_height = CONST.IMAGE_BOTTOM_HEIGHT


def camera_callback(msg):
    global img, img_id
    arr = np.fromstring(msg.data, np.uint8)
    img_id = msg.header.seq
    img = cv.resize(cv.imdecode(arr, 1), (img_width, img_height))


def find_parameter():
    global img
    path = rospkg.RosPack().get_path('zeabus_vision')
    client_name = '/ueye_cam_nodelet_bottom_left/'
    param = 'exposure'
    a = str(raw_input('Press NO. File'))
    f = open(path + "/src/parameter_" + a + ".txt", "a")
    img_id_current = 0
    while not rospy.is_shutdown():
        if img is None:
            print('img is None')
            continue
        exposure_time = rospy.get_param(
            "/" + str(client_name) + str(param), None)

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        Lab = cv.cvtColor(img, cv.COLOR_BGR2Lab)
        YCrCb = cv.cvtColor(img, cv.COLOR_BGR2YCrCb)

        h, s, v = cv.split(hsv)
        L, a, b = cv.split(Lab)
        Y, Cr, Cb = cv.split(YCrCb)

        h, s, v = np.mean(h), np.mean(s), np.mean(v)
        L, a, b = np.mean(L), np.mean(a), np.mean(b)
        Y, Cr, Cb = np.mean(Y), np.mean(Cr), np.mean(Cb)

        h, s, v = '%.2f' % h, '%.2f' % s, '%.2f' % v
        L, a, b = '%.2f' % L, '%.2f' % a, '%.2f' % b
        Y, Cr, Cb = '%.2f' % Y, '%.2f' % Cr, '%.2f' % Cb

        if img_id != img_id_current:
            text = str(img_id) + ' ' + str(h) + ' ' + str(s) + ' ' + str(v)
            text += ' ' + str(L) + ' ' + str(a) + ' ' + str(b)
            text += ' ' + str(Y) + ' ' + str(Cr) + ' ' + str(Cb)
            text += ' ' + str('%.2f' % exposure_time)
            text += '\n'
            print(text)
            f.write(text)
        img_id_current = img_id
        cv.imwrite(path + "/src/img/" + a + '_' + str(img_id) + ".jpg", img)
        cv.waitKey(1000)
    f.close()


if __name__ == '__main__':
    rospy.init_node('exposure_regression')
    print('<' * 8, 'init node', '>' * 8)
    camera_topic = '/top/center/image_raw/compressed'
    rospy.Subscriber(camera_topic, CompressedImage, camera_callback)
    find_parameter()
