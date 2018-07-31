#!/usr/bin/python2

import rospy, math, tf
import constants as cons
from zeabus_elec_ros_hardware_interface.srv import IOCommand, Torpedo
from zeabus_controller.msg import point_xy
from zeabus_controller.srv import *
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Bool

class AIControl:
    def __init__(self):
        self.auv_state = [0, 0, 0, 0, 0, 0]

    def getTarget(self, target):
        print 'Request for %s'%(target)

    def fullSpeed(self, time):
        print 'WARNING: BE READY TO STOP THE CONTROL NODE, IF ANYTHING GOING WRONG'
        self.srv_full_speed(String("SAUV"))
        rospy.sleep(time)
        self.srv_full_speed(String(""))
        print 'AUV stop'

    def listToTwist(self, lis):
        twist = Twist()
        twist.linear.x = lis[0]
        twist.linear.y = lis[1]
        twist.linear.z = lis[2]

        twist.angular.x = lis[3]
        twist.angular.y = lis[4]
        twist.angular.z = lis[5]

        return twist

    def backTrack(self, point):
        print 'Backtracking'

    def multiMove(self, speed):
        print 'MOVE'
        print 'X: %f'%speed[0]
        print 'Y: %f'%speed[1]
        print 'Z: %f'%speed[2]
        print 'YAW: %f'%speed[5]

    def move(self, direction, speed, yaw=0):
        print("speed : %.2f"%(speed))

        print 'Move %s at speed %f m/s, yaw: %f'%(direction, speed, yaw)

    def fixXY(self, x, y, err=0.05, user='mission_planner'):
        print 'Move to (%f, %f)'%(x, y)

    def driveX(self, x, err=0.2, user='mission_planner'):
        print 'doing drive x'

    def driveY(self, y, err=0.2, user='mission_planner'):
        print 'doing drive y'

    def turnRelative(self, degree, err=0.05, user='mission_planner'):
        print 'turning %f'%(degree)

    def turnAbs(self, degree, err=0.05, user='mission_planner'):
        print 'turning to %f'%(degree)

    def depthAbs(self, depth, err=0.2, user='mission_planner'):
        print 'move to depth %f'%(depth)

    def depthRelative(self, depth, err=0, user='mission_planner'):
        print 'move to depth %f'%(depth)

    def stop(self):
        pass

    def gripper(self, cmd):
        if cmd == 'on':
            print 'Release Gripper'
        elif cmd == 'off':
            print 'Hold Gripper'

    def fire(self):
        print 'fire'

    def hold(self):
        print 'hold'

if __name__=='__main__':
    rospy.init_node('aicontrol_node')
