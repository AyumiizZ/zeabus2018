#!/usr/bin/env python2

import rospy
import rosnode

if __name__=='__main__':
    rospy.init_node('test',anonymous=False)
    print(rosnode.rosnode_listnodes())
    while not rospy.is_shutdown():
        print('anonymous=False')
        rospy.sleep(1)