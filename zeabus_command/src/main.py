#!/usr/bin/python2.7
import rospy
from std_msgs.msg import Bool
from aicontrol import AIControl
from gate import Gate
from path2 import Path
from shoot_craps import ShootCraps
from roulette import Roulette
from slots import Slots
from cash_in import CashIn

switch_state = False
count = 0
reset = 0

def getSwitchState(data):
    global switch_state, count
    switch_state = data.data

    if switch_state:
        count += 1

    else: reset += 1

    if count >= 10:
        reset = 0
        count = 0
    elif reset >= 10:
        os.system('rosnode kill /main_ai')


if __name__=='__main__':
    rospy.init_node('main_ai')
    rospy.Subsciber('/planner_switch', Bool, getSwitchState, queue_size=1)
    gate = Gate()
    path1 = Path()
    shoot_craps = ShootCraps()
    path2 = Path()
    slots = Slots()
    roulette = Roulette()
    cash_in = CashIn()

    gate.run()

    path1.run()
    if path1.failed:
        auv.turnAbs(cons.SC_DEGREES)

    shoot_craps.run()

    path2.run()
    if path2.failed:
        auv.turnAbs(cons.SLOTS_DEGREES)

    slots.run()

    roulette.run()

    cash_in.run()

