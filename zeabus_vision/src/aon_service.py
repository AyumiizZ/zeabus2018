#!/usr/bin/env python
import rospy
from zeabus_vision.msg import *
from zeabus_vision.srv import *
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('call_service')

    service_name = ['vision_casino_gate',
                    'vision_path',
                    'vision_dice',
                    'vision_slots',
                    'vision_roulette',
                    'vision_buy_a_gold_chip',
                    'vision_cash_in']
    for i in range(len(service_name)):
        print "[" + str(i+1) + "] " + service_name[i]
    no_service = input("plz input no of service: ")
    serviceName = service_name[no_service-1]
    print('wait service')
    rospy.wait_for_service(serviceName)
    print('service start')
    if no_service == 1:
        call = rospy.ServiceProxy(serviceName, vision_srv_casino_gate)
    elif no_service == 2:
        call = rospy.ServiceProxy(serviceName, vision_srv_path)
    elif no_service == 3:
        call = rospy.ServiceProxy(serviceName, vision_srv_dice)
    elif no_service == 4:
        call = rospy.ServiceProxy(serviceName, vision_srv_slots)
    elif no_service == 5:
        call = rospy.ServiceProxy(serviceName, vision_srv_roulette)
    elif no_service == 6:
        call = rospy.ServiceProxy(serviceName, vision_srv_buy_a_gold_chip)
    elif no_service == 7:
        call = rospy.ServiceProxy(serviceName, vision_srv_cash_in)

    str_call = ['ERROR']
    if no_service == 1:
        str_call = ['casino_gate']
    elif no_service == 2:
        str_call = ['path']
    elif no_service == 3:
        task = ['shoot_craps', 'shoot_craps_hit']
        for i in range(len(task)):
            print "[" + str(i+1) + "] " + task[i]
        no_task = input("plz input no of task: ")
        if no_task == 1 or no_task == 2:
            str_call = [task[no_task-1]]
    elif no_service == 4:
        task = ['yellow_hole', 'red_hole', 'handle']
        for i in range(len(task)):
            print "[" + str(i+1) + "] " + task[i]
        no_task = input("plz input no of task: ")
        if no_task == 1 or no_task == 3:
            str_call = [task[no_task-1]]
        if no_task == 2:
            req = ['big', 'small']
            for i in range(len(req)):
                print "[" + str(i+1) + "] " + req[i]
            no_req = input("plz input no of req: ")
            if no_req == 1 or no_req == 2:
                str_call = [task[no_task-1], req[no_req-1]]
    elif no_service == 5:
        req = ['red', 'green', 'black', 'center']
        for i in range(len(req)):
            print "[" + str(i+1) + "] " + req[i]
        no_req = input("plz input no of req: ")
        if no_req == 1 or no_req == 2 or no_req == 3:
            str_call = ['roulette', req[no_req-1]]
    elif no_service == 6:
        req = ['front', 'bottom']
        for i in range(len(req)):
            print "[" + str(i+1) + "] " + req[i]
        no_req = input("plz input no of req: ")
        if no_req == 1 or no_req == 2:
            str_call = ['buy_a_gold_chip', req[no_req-1]]
    elif no_service == 7:
        task = ['cash_in', 'cash_in_top', 'cash_in_bottom']
        for i in range(len(task)):
            print "[" + str(i+1) + "] " + task[i]
        no_task = input("plz input no of task: ")
        if no_task == 1:
            str_call = [task[no_task-1], 'bin']
        if no_task == 2 or no_task == 3:
            req = ['yellow', 'red', 'green']
            for i in range(len(req)):
                print "[" + str(i+1) + "] " + req[i]
            no_req = input("plz input no of req: ")
            if no_req == 1 or no_req == 2 or no_req == 3:
                str_call = [task[no_task-1], req[no_req-1]]
    while not rospy.is_shutdown():
        if len(str_call) == 1:
            res = call(String(str_call[0]))
        elif len(str_call) == 2:
            res = call(String(str_call[0]), String(str_call[1]))
        print res

        rospy.sleep(0.1)
