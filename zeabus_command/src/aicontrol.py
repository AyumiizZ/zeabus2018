#!/usr/bin/python2


import rospy, math, tf
import constants as cons
from zeabus_elec_ros_hardware_interface.srv import IOCommand
from zeabus_controller.msg import point_xy
from zeabus_controller.srv import *
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Bool

class AIControl:

    def __init__(self):
        self.vel = Twist()
        self.pose = Pose()
        self.point = Point()

        self.auv_state = [0, 0, 0, 0, 0, 0]
        rospy.Subscriber('/auv/state', Odometry, self.getState)
        # real
        self.pub_vel = rospy.Publisher('/zeabus/cmd_vel', Twist, queue_size=10)
        # sim
        #self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        '''
        self.pub_position = rospy.Publisher('/cmd_fix_position', Point, queue_size=10)
        self.pub_xy = rospy.Publisher('/fix/abs/xy', point_xy, queue_size=10)

        self.pub_abs_yaw = rospy.Publisher('/fix/abs/yaw', Float64, queue_size=10)
        self.pub_rel_yaw = rospy.Publisher('/fix/rel/yaw', Float64, queue_size=10)
        self.pub_abs_depth = rospy.Publisher('/fix/abs/depth', Float64, queue_size=10)
        self.pub_rel_depth = rospy.Publisher('/fix/rel/depth', Float64, queue_size=10)
        '''
        '''
        rospy.wait_for_service('io_and_pressure/IO_ON')
        print 'IO_ON'
        rospy.wait_for_service('io_and_pressure/IO_OFF')
        print 'IO_OFF'
        '''
        rospy.wait_for_service('fix_abs_yaw')
        print 'abs_yaw'
        rospy.wait_for_service('fix_rel_yaw')
        print 'rel_yaw'
        rospy.wait_for_service('fix_rel_xy')
        print 'rel_xy'
        rospy.wait_for_service('fix_abs_xy')
        print 'abs_xy'
        rospy.wait_for_service('fix_abs_depth')
        print 'abs_depth'
        rospy.wait_for_service('fix_rel_depth')
        print 'rel_depth'
        rospy.wait_for_service('ok_position')
        print 'ok_position'
        #rospy.wait_for_service('fix_service')
        #print 'fix_service'

        #self.srv_io_on = rospy.ServiceProxy('io_and_pressure/IO_ON', IOCommand)
        #self.srv_io_off = rospy.ServiceProxy('io_and_pressure/IO_OFF', IOCommand)
        self.srv_abs_yaw = rospy.ServiceProxy('fix_abs_yaw', fix_abs_yaw)
        self.srv_rel_yaw = rospy.ServiceProxy('fix_rel_yaw', fix_abs_yaw)
        self.srv_rel_xy = rospy.ServiceProxy('fix_rel_xy', fix_rel_xy)
        self.srv_abs_xy = rospy.ServiceProxy('fix_abs_xy', fix_abs_xy)
        self.srv_abs_depth = rospy.ServiceProxy('fix_abs_depth', fix_abs_depth)
        self.srv_rel_depth = rospy.ServiceProxy('fix_rel_depth', fix_abs_depth)
        self.srv_ok_position = rospy.ServiceProxy('ok_position', ok_position)
        #self.srv_full_speed = rospy.ServiceProxy('fix_service', message_service)

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
        self.driveX(-2)
        self.fixXY(point[0], point[1])
        print 'Back to checkpoint'

    def multiMove(self, speed):
        self.stop()
        temp = self.listToTwist(speed)
        print 'MOVE'
        print 'X: %f'%speed[0]
        print 'Y: %f'%speed[1]
        print 'Z: %f'%speed[2]
        print 'YAW: %f'%speed[5]
        for _ in range(3):
            self.pub_vel.publish(temp)

    def move(self, direction, speed, yaw=0):
        print("speed : %.2f"%(speed))
        self.stop()

        print 'Move %s at speed %f m/s, yaw: %f'%(direction, speed, yaw)

        if direction == 'left':
           self.vel.linear.y = speed
        elif direction == 'right':
            self.vel.linear.y = -speed
        elif direction == 'forward':
            self.vel.linear.x = speed
        elif direction == 'backward':
            self.vel.linear.x = -speed
        elif direction == 'up' :
            self.vel.linear.z = speed
        elif direction == 'down' :
            self.vel.linear.z = -speed

        self.vel.angular.z = yaw

        for _ in range(3):
            self.pub_vel.publish(self.vel)

    def fixXY(self, x, y, err=0, user='mission_planner'):
        print 'Move to (%f, %f)'%(x, y)
        self.srv_abs_xy(x, y, String(user))
        while not rospy.is_shutdown() and not self.srv_ok_position(String('xy'), err, String(user)).ok:
            rospy.sleep(0.1)

        print 'finish moving'

    def driveX(self, x, err=0.2, user='mission_planner'):
        print 'doing drive x'
        self.stop()
        count = 0
        reset = 0
        while not rospy.is_shutdown() and count < 10:
            if self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
                rospy.sleep(0.2)
            else: reset += 1
            if reset >= 5: 
                count = 0
                reset = 0
            print count
            print 'Wait for xy'
        print('driveX already')
        self.srv_rel_xy(x, 0, String(user))
        count = 0

        while not rospy.is_shutdown() and count < 10:
            if self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
                rospy.sleep(0.2)
            print count

        print 'finish drive x'

    def driveY(self, y, err=0.2, user='mission_planner'):
        print 'doing drive y'
        self.stop()
        count = 0
        reset = 0
        while not rospy.is_shutdown() and count < 10:
            check_pos = self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok
            print 'pos: %s'%check_pos
            if check_pos:
                count += 1
                print 'Wait for xy'
                rospy.sleep(0.2)
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
        print('driveY already')
        self.srv_rel_xy(0, y, String(user))

        while not rospy.is_shutdown() and count < 10:
            check_pos = self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok
            print 'pos: %s'%check_pos
            if check_pos:
                rospy.sleep(0.1)
                count += 1

        print 'finish drive y'

    def turnRelative(self, degree, err=0.05, user='mission_planner'):
        self.stop()
        print 'turning %f'%(degree)
        count = 0
        reset = 0
        rospy.sleep(1.5)
        while not rospy.is_shutdown() and count < 10:
            check_pos = self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err,String(user)).ok
            print 'pos: %s'%check_pos
            if check_pos:
                print 'if'
                count += 1
                rospy.sleep(0.2)
            else: 
                print 'else'
                reset += 1

            if reset >= 5:
                count = 0
                reset = 0
            print 'wait xy yaw'
            print count

        print('turn already')
        rand = math.radians(degree)
        self.srv_rel_yaw(rand, String(user))
        count = 0
        reset = 0
        rospy.sleep(1.5)
        while not rospy.is_shutdown() and count < 10:
            check_pos = self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok
            if check_pos:
                print 'if'
                print 'pos: %s'%check_pos
                count += 1
                print('Wait for yaw')
                rospy.sleep(0.2)
            elif not check_pos:
                print 'else'
                print 'pos: %s'%check_pos
                reset = 0

            if reset >= 5:
                count = 0
                reset = 0
            print count

        print 'finish turn rel'

    def turnAbs(self, degree, err=0.05, user='mission_planner'):
        self.stop()
        print 'turning to %f'%(degree)
        count = 0
        reset = 0
        rospy.sleep(1.2)
        while not rospy.is_shutdown() and count < 10:
            if self.srv_ok_position(String('xy'), err, String(user)).ok  and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
                rospy.sleep(0.2)
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            print 'wait xy yaw'
            print count

        print('turn already')
        rand = math.radians(degree)
        self.srv_abs_yaw(rand, String(user))


        count = 0
        reset = 0
        rospy.sleep(1.2)
        while not rospy.is_shutdown() and count < 10:
            if self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
                rospy.sleep(0.2)
            else: reset += 1
            if reset >= 5: 
                count = 0
                reset = 0
            print count

        print 'finish turn abs'

    def depthAbs(self, depth, err=0.2, user='mission_planner'):
        self.stop()
        print 'move to depth %f'%(depth)

        count = 0
        reset = 0

        while not rospy.is_shutdown() and count < 10:
            if self.srv_ok_position(String('z'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            print count

        print 'READY Z'
        self.srv_abs_depth(depth, String(user))
        count = 0
        reset = 0

        while not rospy.is_shutdown() and count < 10:
            if self.srv_ok_position(String('z'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            print count
            print 'wait for z'
            rospy.sleep(0.4)

        print 'finish depth abs'

    def depthRelative(self, depth, err=0, user='mission_planner'):
        self.stop()
        print 'move to depth %f'%(depth)
        count = 0
        reset = 0
        while not rospy.is_shutdown() and count < 10:
            if self.srv_ok_position(String('z'), err, String(user)).ok :
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            print count
        print 'depth already'
        self.srv_rel_depth(depth, String(user))
        count = 0
        reset = 0

        while not rospy.is_shutdown() and count < 10:
            if self.srv_ok_position(String('z'), err, String(user)).ok:
                count += 1
                rospy.sleep(0.4)
            else:reset += 1
            if reset >= 5: 
                reset = 0
                count = 0
            print count

        print'finish depth relative'

    def stop(self):
        rospy.sleep(0.3)
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0

        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        for _ in range(3):
            self.pub_vel.publish(self.vel)

    def gripper(self, cmd):
        if cmd == 'on':
            self.srv_io_on(5)
        elif cmd == 'off':
            self.srv_io_off(5)

    def getState(self, data):
        self.pose = data.pose.pose
        pose = self.pose

        temp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler_angular = tf.transformations.euler_from_quaternion(temp)

        self.auv_state[0] = pose.position.x
        self.auv_state[1] = pose.position.y
        self.auv_state[2] = pose.position.z

        self.auv_state[3] = euler_angular[0]
        self.auv_state[4] = euler_angular[1]
        self.auv_state[5] = euler_angular[2]

if __name__=='__main__':
    rospy.init_node('aicontrol_node')
    aicontrol = AIControl()
    auv = aicontrol
    print 'done'