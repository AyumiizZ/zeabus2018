#!/usr/bin/python2

import rospy , math
from std_msgs.msg       import String
from control_auv		import auv_control
from zeabus_vision.msg	import vision_cash_in
from zeabus_vision.srv	import vision_srv_cash_in

class go_cash_in_your_chip:

    def __init__(self):
#------------------ setup service for call data from vision -------------------------
        print "wait service /vision_cash_in_your_chip"
        self.information 	= rospy.ServiceProxy('vision_cash_in' , vision_srv_cash_in)
        print "have service /vision_cash_in_your_chip"
#-----------------------------------------------------------------------------------------

    def main(self):

if __name__=='__main__':
    rospy.init_node("go_cash_in_your_chip")
    play_cash = go_cash_in_your_chip( 5.1 , 0 , 0 , -0.1)
    play_cash.main(2)
