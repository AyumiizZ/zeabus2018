#!/usr/bin/env python2

import rospy


if __name__=='__main__':
    rospy.init_node('test',anonymous=True)
    while not rospy.is_shutdown():
        print('anonymous=True')
        rospy.sleep(1)