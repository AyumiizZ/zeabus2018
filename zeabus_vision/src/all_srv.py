#!/usr/bin/python2.7
import rospy
from zeabus_vision.msg import *
from zeabus_vision.srv import *
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('call_service')
    service_name = ['vision_casino_gate',
                    'vision_path',
                    'vision_shoot_craps',
                    'vision_slots',
                    'vision_roulette',
                    'vision_buy_a_gold_chip',
                    'vision_cash_in']
    print('wait service')
    for name in service_name:
        print "srv: "+name
        rospy.wait_for_service(name)
        print "success"
    print('service start')

    call1 = rospy.ServiceProxy(service_name[0], vision_srv_casino_gate)
    call2 = rospy.ServiceProxy(service_name[1], vision_srv_path)
    call3 = rospy.ServiceProxy(service_name[2], vision_srv_dice)
    call4 = rospy.ServiceProxy(service_name[3], vision_srv_slots)
    call5 = rospy.ServiceProxy(service_name[4], vision_srv_roulette)
    call6 = rospy.ServiceProxy(service_name[5], vision_srv_buy_a_gold_chip)
    call7 = rospy.ServiceProxy(service_name[6], vision_srv_cash_in)
    i = 1
    while not rospy.is_shutdown():
        res1 = call1(String('casino_gate'))
        res2 = call2(String('path'))
        # res3_1 = call3(String('shoot_craps'))
        # res3_2 = call3(String('shoot_craps_hit'))
        res4_1 = call4(String('yellow_hole'),String('N/A'))
        res4_2 = call4(String('red_hole'),String('big'))
        res4_3 = call4(String('red_hole'),String('small'))
        res4_4 = call4(String('handle'),String('N/A'))
        res5 = call5(String('roulette'),String('green'))
        res5 = call5(String('roulette'),String('red'))
        res5 = call5(String('roulette'),String('black'))
        res6_1 = call6(String('buy_a_gold_chip'),String('front'))
        res6_2 = call6(String('buy_a_gold_chip'),String('bottom'))
        res7_1 = call7(String('cash_in'),String('bin'))
        # res7_2 = call7(String('cash_in_top'),String('red'))
        # res7_3 = call7(String('cash_in_top'),String('green'))
        res7_4 = call7(String('cash_in_top'),String('yellow'))
        # res7_5 = call7(String('cash_in_bottom'),String('red'))
        # res7_6 = call7(String('cash_in_bottom'),String('green'))
        # res7_7 = call7(String('cash_in_bottom'),String('yellow'))
        print "success "+str(i)+" loop"
        i+=1
        rospy.sleep(0.1)
