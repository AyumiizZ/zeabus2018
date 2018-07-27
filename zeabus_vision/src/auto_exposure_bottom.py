#!/usr/bin/env python
import rospy
from auto_exposure import AutoExposure


def main():
    EVmin = 0.4
    EVdefault = 0.7
    subTopicL = rospy.get_param(
        "/auto_exposure_bottom/imageTopicL", None)
    clientL = rospy.get_param(
        "/auto_exposure_bottom/imageClientL", None)
  
   
    if not subTopicL is None:
        AEL = AutoExposure(subTopicL, clientL, EVdefault, EVmin, 'bottom')
        AEL.adjust_exposure_time()


if __name__ == '__main__':
    rospy.init_node('Auto_Exposure_Bottom')
    main()
