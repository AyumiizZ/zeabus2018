#!/usr/bin/python2

import rospy
from zeabus_vision.msg import vision_slots
from zeabus_vision.srv import vision_srv_slots
from aicontrol import AIControl
from std_msgs.msg import String, Float64
import constants as cons

debug = False

class Slots(object):

    def __init__(self):
        self.data = vision_slots()
        self.aicontrol = AIControl()

        if debug:
            print 'Debug mode is activated'

        print '<====INIT SLOTS===>'
        print 'wait for vision service'
        rospy.wait_for_service('vision_slots')
        self.detect_slots = rospy.ServiceProxy('vision_slots', vision_srv_slots)

    def detectSlots(self, task, req):
        try:
            self.data = self.detect_slots(String(task), String(req))
            self.data = self.data.data
        except: print 'Error'

    def findHandleFront(self):
        self.detectSlots('handle', 'front')

    def getData(self):
        return self.data.appear, self.data.cx, self.data.cy, self.data.right_excess, self.data.area

    def saveCheckpoint(self):
        point = self.aicontrol.auv_state
        x = point[0]
        y = point[1]
        z = point[2]
        yaw = point[5]

        print '====SAVE POINT===='
        print 'X: %f'%(x)
        print 'Y: %f'%(y)
        print 'Z: %f'%(z)
        print 'Yaw: %f'%(yaw)
        print '=================='
        return x, y, z, yaw

    def run(self):
        global debug

        auv = self.aicontrol

        #auv.depthAbs(-3)

        count = 0
        reset = 0
        fail = 0
        redo = 0

        target = ['yellow_hole', 'red_hole', 'red_hole']
        size = ['', 'small', 'big']
        piority = 0
        mode = 0

        last_cx = 0
        last_cy = 0
        last_area = 0

        pos = 1

        start_x, start_y, start_z, yaw = self.saveCheckpoint()
        check_x, check_y, check_z, check_yaw = 0, 0, 0, 0

        while not rospy.is_shutdown() and mode != -1:
            if mode <= 3:
                self.findHandleFront()
            elif mode >= 4:
                if piority < 3:
                    task = target[piority]
                    req = size[piority]
                self.detectSlots(task, req)

            appear, cx, cy, right_excess, area = self.getData()
            if appear:
                last_cx = cx
                last_cy = cy
                last_area = area

            #find handle
            if mode == 0:
                print 'Mode 0'

                if fail >= 10:
                    print 'FAILED MODE 0'
                    auv.fixXY(start_x, start_y)
                    #auv.turnAbs(cons.SLOTS_DEGREES)
                    redo += 1
                    fail = -10

                if appear:
                    count += 1
                    reset = 0
                    print 'Found handle: %d'%(count)
                else:
                    print 'Not found handle: %d'%(reset)
                    auv.move('forward', cons.AUV_L_SPEED)
                    reset += 1

                if count >= 10:
                    mode = 1
                    count = 0
                    reset = 0
                    fail = 0
                    print 'Found handle'

                if reset >= 5:
                    fail += 1
                    count = 0
                    reset = 0


            #get handle to the middle
            if mode == 1:
                print 'Mode 1'
                if appear:
                    if cons.VISION_HAND_CY - 0.5 <= cy <= cons.VISION_HAND_CY + 0.5:
                        auv.multiMove([0, 0, (cy - cons.VISION_HAND_CY) * cons.AUV_L_SPEED, 0, 0, 0])
                        if abs(cx) < cons.VISION_HAND_ERROR:
                            auv.multiMove([0, -cx, 0, 0, 0, 0])
                            count += 1
                            print 'Center'
                        elif cx != 0:
                            print 'fixing x'
                            auv.multiMove([0, -cx, 0, 0, 0, 0])
                    elif cy != cons.VISION_HAND_CY:
                        print 'fixing y'
                        print (cy)
                        auv.multiMove([0, 0, (cy - cons.VISION_HAND_CY)*cons.AUV_L_SPEED, 0, 0, 0])

                else:
                    auv.multiMove([0, (-last_cx) * cons.AUV_L_SPEED, last_cy * cons.AUV_L_SPEED, 0, 0, 0])
                    fail += 1

                if count >= 5:
                    mode = 2
                    count = 0
                    reset = 0
                    fail = 0
                if reset >= 5:
                    reset = 0
                    count = 0

                if fail // 4 >= 10:
                    print 'FAILED MODE 1'
                    mode = 0
                    count = 0
                    reset = 0

            #get to pulling position
            if mode == 2:
                print '======================'
                print 'MODE 2'
                '''
                print 'cx:%f'%(cx)
                print 'cy:%f'%(cy)
                print 'appear:%s'%(appear)
                print 'area:%f'%(area)
                '''
                if appear:
                    fail = 0
                    if area <= 0.005 and cons.VISION_HAND_CY - 0.1 <= cy <= cons.VISION_HAND_CY + 0.06 and cons.VISION_HAND_CX - 0.06 <= cx <= cons.VISION_HAND_CX + 0.2:
                        auv.move('forward', cons.AUV_M_SPEED)
                    elif ((cons.VISION_HAND_CY - 0.1 > cy or cy > cons.VISION_HAND_CY + 0.06) or (cons.VISION_HAND_CX - 0.06 > cx or cons.VISION_HAND_CX + 0.2)):
                        if right_excess and area >= 0.25:
                            if cx < 0.5:
                                for _ in range(5):
                                    auv.move('left', cons.AUV_L_SPEED)
                                count = 10
                            count += 1

                        if cons.VISION_HAND_CY - 0.1 <= cy <= cons.VISION_HAND_CY + 0.06:
                            if cons.VISION_HAND_CX - 0.06 <= cx <= cons.VISION_HAND_CX + 0.2:
                                if not right_excess or area < 0.15:
                                    auv.move('forward', cons.AUV_M_SPEED * (0.15 - area))
                                    reset += 1
                                elif right_excess and area >= 0.15:
                                    print 'Handle in position'
                                    rospy.sleep(1)
                                    count += 1
                                    reset = 0

                            elif cx != cons.VISION_HAND_CX:
                                print 'CX not in position'
                                auv.multiMove([0, (cons.VISION_HAND_CX - cx) * cons.AUV_M_SPEED, (cy - cons.VISION_HAND_CY) * cons.AUV_L_SPEED, 0, 0, 0])
                                #auv.multiMove([0, (cons.VISION_HAND_CX - cx)*cons.AUV_L_SPEED, 0, 0, 0, 0])

                        elif cy != cons.VISION_HAND_CY:
                            auv.multiMove([0, (cons.VISION_HAND_CX - cx)*cons.AUV_L_SPEED, (cy - cons.VISION_HAND_CY) * cons.AUV_L_SPEED, 0, 0, 0])
                            #auv.multiMove([0, 0, (cy - cons.VISION_HAND_CY)*cons.AUV_M_SPEED, 0, 0, 0])
                else:
                    #auv.move('down', cons.AUV_L_SPEED)
                    fail += 1

                if count >= 10:
                    mode = 3
                    #mode = -1
                    count = 0
                    reset = 0
                    fail = 0
                    for _ in range(5):
                        auv.move('down', cons.AUV_M_SPEED)
                    check_x, check_y, check_z, check_yaw = self.saveCheckpoint()
                if reset >= 3:
                    count = 0
                    reset = 0

                if fail // 4 >= 10:
                    print 'FAILED'
                    mode = 0
                    count = 0
                    reset = 0

            #pull arm
            if mode == 3:
                print 'MODE 3'
                while not rospy.is_shutdown() and check_z - auv.auv_state[2] < 0.30:
                    for _ in range(5):
                        self.findHandleFront()
                        cx = self.data.cx
                        appear = self.data.appear
                        if appear:
                            if cx < 0.5:
                                auv.move('left', cons.AUV_L_SPEED)

                    for _ in range(5):
                        auv.move('down', cons.AUV_M_SPEED)
                        #auv.multiMove([0, (- cx) * cons.AUV_M_SPEED, cons.AUV_M_SPEED, 0, 0, 0])
                    for _ in range(3):
                        auv.move('backward', cons.AUV_L_SPEED)
                        #auv.multiMove([cons.AUV_L_SPEED, (- cx) * cons.AUV_M_SPEED, 0, 0, 0, 0])

                #mode = -1

                auv.depthAbs(check_z)
                ###########
                # no dvl
                ###########
                for _ in range(5):
                    auv.move('backward', cons.AUV_M_SPEED)

                while not rospy.is_shutdown() and not appear:
                    task = target[piority]
                    req = size[piority]
                    self.detectSlots(task, req)

                    appear, cx, cy, right_excess, area = self.getData()
                    auv.move('left', cons.AUV_L_SPEED)

                ###########

                ###########
                # with dvl
                ###########
                '''
                auv.fixXY(check_x, check_y)
                auv.driveX(-0.3)
                auv.driveY(0.3)
                '''
                ###########
                mode = 4

            #aim torpedo
            if mode == 4:

                print 'MODE 4'
                print 'Target is %s %s'%(size[piority], target[piority])
                print 'CX: %f'%(cx)
                print 'CY: %f'%(cy)
                if appear:
                    if cx > 0:
                        pos = -1
                    elif cx < 0:
                        pos = 1
                    print 'Found'
                    fail = 0
                    if area < 0.05:
                        auv.multiMove([((cons.AUV_H_SPEED - 0.2) * (cons.VISION_SLOTS_AREA - area)) + 0.03, cons.AUV_L_SPEED * (cons.VISION_TORPEDO_CX - cx), cons.AUV_L_SPEED * (cy - cons.VISION_TORPEDO_CY), 0, 0, 0])
                    elif area > 0.06:
                        auv.multiMove([((cons.AUV_H_SPEED - 0.2) * (cons.VISION_SLOTS_AREA - area)) - 0.03, cons.AUV_L_SPEED * (cons.VISION_TORPEDO_CX - cx), cons.AUV_L_SPEED * (cy - cons.VISION_TORPEDO_CY), 0, 0, 0])

                    elif 0.05 <= area <= 0.06:
                        if cons.VISION_TORPEDO_CX - 0.07 <= cx <= cons.VISION_TORPEDO_CX + 0.07 and cons.VISION_TORPEDO_CY - 0.1 <= cy <= cons.VISION_TORPEDO_CY + 0.1:
                            print 'Ready to fire torpedo: %d'%(count)
                            auv.multiMove([0, cons.AUV_L_SPEED * (cons.VISION_TORPEDO_CX - cx), cons.AUV_H_SPEED * (cy - cons.VISION_TORPEDO_CY), 0, 0, 0])
                            #auv.multiMove([0, (cons.AUV_H_SPEED - 0.2) * (-cx), (cons.AUV_H_SPEED - 0.2) * cy, 0, 0, 0])
                            count += 1
                            reset = 0
                        else:
                            print 'Get in position: %d'%(reset)
                            auv.multiMove([0, cons.AUV_M_SPEED * (cons.VISION_TORPEDO_CX - cx), cons.AUV_H_SPEED * (cy - cons.VISION_TORPEDO_CY), 0, 0, 0])
                            reset += 1

                elif not self.data.appear and cx == -1 and cy == -1:
                    if last_area < 0.05:
                        auv.multiMove([((cons.AUV_H_SPEED - 0.2) * (cons.VISION_SLOTS_AREA - last_area)) + 0.03, cons.AUV_L_SPEED * last_cx, cons.AUV_M_SPEED * last_cy, 0, 0, 0])
                    elif last_area > 0.06:
                        auv.multiMove([((cons.AUV_H_SPEED - 0.2) * (cons.VISION_SLOTS_AREA - last_area)) - 0.03, cons.AUV_L_SPEED * last_cx, cons.AUV_M_SPEED * last_cy, 0, 0, 0])

                    #fail += 1

                if count >= 5:
                    auv.fire()
                    print 'FIREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE'
                    mode = -1
                    print 'Done'
                if reset >= 5:
                    count = 0
                    reset = 0

                '''
                if fail//4 >= 10:
                    print 'CAN\'t FIND TARGET'
                    if piority <= 2:
                        print 'Find next target %s %s'%(target[piority], size[piority])
                        auv.driveX(-1)
                        piority += 1
                        fail = 0
                    if piority > 2:
                        print 'Can\'t find any target fire randomly'
                        #auv.fire()
                '''

            if redo >= 3:
                print 'Abort Mission'
                mode = -1

        if debug:
            rospy.sleep(2)
        rospy.sleep(0.2)

if __name__=='__main__':
    rospy.init_node('slots_node')
    slots = Slots()
    slots.run()
