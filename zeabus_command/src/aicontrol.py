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
        self.vel = Twist()
        self.pose = Pose()
        self.point = Point()

        self.auv_state = [0, 0, 0, 0, 0, 0]
        self.checkpoint = [0, 0, 0, 0, 0, 0]
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
        print 'wait for fix_abs_yaw'
        rospy.wait_for_service('fix_abs_yaw')
        print 'abs_yaw'
        print 'wait for fix_rel_yaw'
        rospy.wait_for_service('fix_rel_yaw')
        print 'rel_yaw'
        print 'wait for fix_rel_xy'
        rospy.wait_for_service('fix_rel_xy')
        print 'rel_xy'
        print 'wait for fix_abs_xy'
        rospy.wait_for_service('fix_abs_xy')
        print 'abs_xy'
        print 'wait for fix_abs_depth'
        rospy.wait_for_service('fix_abs_depth')
        print 'abs_depth'
        print 'wait for fix_rel_depth'
        rospy.wait_for_service('fix_rel_depth')
        print 'rel_depth'
        print 'wait for ok_position'
        rospy.wait_for_service('ok_position')
        print 'ok_position'
        #rospy.wait_for_service('fix_service')
        #print 'fix_service'
        print 'wait for know_target'
        rospy.wait_for_service('know_target')
        print 'know_target'
        print 'wait for fire_torpedo'
        rospy.wait_for_service('fire_torpedo')
        print 'fire_torpedo'
        print 'wait for hold_torpedo'
        rospy.wait_for_service('hold_torpedo')
        print 'hold_torpedo'

        #self.srv_io_on = rospy.ServiceProxy('io_and_pressure/IO_ON', IOCommand)
        #self.srv_io_off = rospy.ServiceProxy('io_and_pressure/IO_OFF', IOCommand)
        self.srv_abs_yaw = rospy.ServiceProxy('fix_abs_yaw', fix_abs_yaw)
        self.srv_rel_yaw = rospy.ServiceProxy('fix_rel_yaw', fix_abs_yaw)
        self.srv_rel_xy = rospy.ServiceProxy('fix_rel_xy', fix_rel_xy)
        self.srv_abs_xy = rospy.ServiceProxy('fix_abs_xy', fix_abs_xy)
        self.srv_abs_depth = rospy.ServiceProxy('fix_abs_depth', fix_abs_depth)
        self.srv_rel_depth = rospy.ServiceProxy('fix_rel_depth', fix_abs_depth)
        self.srv_ok_position = rospy.ServiceProxy('ok_position', ok_position)
        self.fire_torpedo = rospy.ServiceProxy('fire_torpedo', Torpedo)
        self.hold_torpedo = rospy.ServiceProxy('hold_torpedo', Torpedo)
        self.target = rospy.ServiceProxy('know_target', target_service)
        #self.srv_full_speed = rospy.ServiceProxy('fix_service', message_service)

    def getTarget(self, target):
        print 'Request for %s'%(target)
        data = self.target(String(target))
        return [data.target_01, data.target_02]


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
        x = point[0]
        y = point[1]
        self.fixXY(x, y)
        print 'Back to checkpoint'

    def multiMove(self, speed):
        self.stop()
        if speed[2] > 0:
            speed[2] += 0.1
        elif speed[2] < 0:
            speed[2] -= 0.1
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

    def fixXY(self, x, y, err=0.05, user='mission_planner'):
        print 'Move to (%f, %f)'%(x, y)
        self.srv_abs_xy(x, y, String(user))

        count = 0
        reset = 0

        cur_x = self.auv_state[0]
        cur_y = self.auv_state[1]

        time_limit = ((5*math.sqrt(((x - cur_x)**2 + (y - cur_y)**2)) // 1) + 5) * 4
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5: 
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        self.srv_rel_xy(x, 0, String(user))
        self.checkpoint = self.auv_state

        count = 0
        reset = 0
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        print 'finish moving'
        self.checkpoint = self.auv_state

    def driveX(self, x, err=0.2, user='mission_planner'):
        print 'doing drive x'
        self.stop()
        count = 0
        reset = 0

        if x < 1:
            time_limit = 40
        else:
            time_limit = ((5 * x) // 1) * 4
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5: 
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        print('driveX already')
        self.srv_rel_xy(x, 0, String(user))
        count = 0
        self.checkpoint = self.auv_state

        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5: 
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        print 'finish drive x'

    def driveY(self, y, err=0.2, user='mission_planner'):
        print 'doing drive y'
        self.stop()
        count = 0
        reset = 0

        if y < 1:
            time_limit = 40
        else:
            time_limit = ((5 * y) // 1) * 4
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            check_pos = self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok
            print 'pos: %s'%check_pos
            if check_pos:
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
        print('driveY already')
        self.srv_rel_xy(0, y, String(user))
        self.checkpoint = self.auv_state

        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            check_pos = self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok
            print 'pos: %s'%check_pos
            if check_pos:
                count += 1
            else: reset += 1
            if reset >= 5: 
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
        print 'finish drive y'

    def turnRelative(self, degree, err=0.05, user='mission_planner'):
        self.stop()
        print 'turning %f'%(degree)
        count = 0
        reset = 0

        rospy.sleep(1.5)

        time_limit = 100
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            check_pos = self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err,String(user)).ok
            print 'pos: %s'%check_pos
            if check_pos:
                count += 1
            else:
                reset += 1

            if reset >= 5:
                count = 0
                reset = 0
            time_check += 1
            print ('time check: %d')%(time_check)
            rospy.sleep(0.2)
            if time_check >= time_limit:
                print 'Timeout'

        print('turn already')
        rand = math.radians(degree)
        self.srv_rel_yaw(rand, String(user))
        count = 0
        reset = 0

        rospy.sleep(1.5)

        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            check_pos = self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok
            if check_pos:
                print 'pos: %s'%check_pos
                count += 1
            elif not check_pos:
                reset = 0

            if reset >= 5:
                count = 0
                reset = 0
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        print 'finish turn rel'
        self.checkpoint = self.auv_state

    def turnAbs(self, degree, err=0.05, user='mission_planner'):
        self.stop()
        print 'turning to %f'%(degree)
        count = 0
        reset = 0

        rospy.sleep(1.5)
        time_limit = 100
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('xy'), err, String(user)).ok  and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            #print 'wait xy yaw'
            #print count

        print('turn already')
        rand = math.radians(degree)
        self.srv_abs_yaw(rand, String(user))


        count = 0
        reset = 0

        rospy.sleep(1.5)
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('xy'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5: 
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        print 'finish turn abs'
        self.checkpoint = self.auv_state

    def depthAbs(self, depth, err=0.2, user='mission_planner'):
        self.stop()
        print 'move to depth %f'%(depth)

        count = 0
        reset = 0

        cur_z = self.auv_state[2]

        time_limit = (5 * abs(cur_z - depth)) * 4
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('z'), err, String(user)).ok and self.srv_ok_position(String('yaw'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        print 'READY Z'
        self.srv_abs_depth(depth, String(user))

        count = 0
        reset = 0

        time_check = 0

        while not rospy.is_shutdown() and count < 3:
            if self.srv_ok_position(String('z'), err, String(user)).ok:
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        print 'finish depth abs'

    def depthRelative(self, depth, err=0, user='mission_planner'):
        self.stop()
        print 'move to depth %f'%(depth)

        count = 0
        reset = 0

        time_limit = (5 * abs(depth)) * 4
        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('z'), err, String(user)).ok :
                count += 1
            else: reset += 1
            if reset >= 5:
                count = 0
                reset = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
        print 'depth already'
        self.srv_rel_depth(depth, String(user))

        count = 0
        reset = 0

        time_check = 0

        while not rospy.is_shutdown() and count < 3 and time_check <= time_limit:
            if self.srv_ok_position(String('z'), err, String(user)).ok:
                count += 1
            else:reset += 1
            if reset >= 5: 
                reset = 0
                count = 0
            rospy.sleep(0.2)
            time_check += 1
            print ('time check: %d')%(time_check)
            if time_check >= time_limit:
                print 'Timeout'
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

    def fire(self):
        print 'fire'
        self.fire_torpedo()
        rospy.sleep(0.5)
        self.hold()

    def hold(self):
        print 'hold'
        self.hold_torpedo()

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
    auv.fire()
    auv.hold()
