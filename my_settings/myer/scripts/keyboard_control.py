
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String


MAX_LINEAR = 20
MAX_ANG_VEL = 3
LINEAR_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.01

cmd_vel_mask = False
ctrl_leader = False

msg2all = """
C
"""

msg2leader = """
C
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_msg():
    if ctrl_leader:
        print(msg2leader)
    else:
        print(msg2all)      

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)
    formation_configs = ['stop controlling']
    
    cmd= String()
    twist = Twist()   
    
    rospy.init_node('iris_multirotor_keyboard_control')
    
    multi_cmd_vel_flu_pub = [None]*1
    multi_cmd_pub = [None]*1
    for i in range(1):
        multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/iris_'+str(i)+'/cmd_vel_flu', Twist, queue_size=1)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/iris_'+str(i)+'/cmd',String,queue_size=3)
    leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)
    leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)

    forward  = 0.3
    leftward  = 0.0
    upward  = 0.6
    angular = 0.0

    print_msg()
    while(1):
        key = getKey()
        if key == 'w' :
            forward = forward + LINEAR_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'x' :
            forward = forward - LINEAR_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'a' :

            leftward = leftward + LINEAR_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'd' :
            leftward = leftward - LINEAR_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'i' :
            upward = upward + LINEAR_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == ',' :
            upward = upward - LINEAR_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'j':
            angular = angular + ANG_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'l':
            angular = angular - ANG_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'r':
            cmd = 'AUTO.RTL'
            print_msg()
            print('Returning home')
        elif key == 't':
            cmd = 'ARM'
            print_msg()
            print('Arming')
        elif key == 'y':
            cmd = 'DISARM'
            print_msg()
            print('Disarming')
        elif key == 'v':
            cmd = 'AUTO.TAKEOFF'
            print_msg()
            #print('Takeoff mode is disenabled now')
        elif key == 'b':
            cmd = 'OFFBOARD'
            print_msg()
            print('Offboard')
        elif key == 'n':
            cmd = 'AUTO.LAND'
            print_msg()
            print('Landing')
        elif key == 'g':
            ctrl_leader = not ctrl_leader
            print_msg()
        elif key in ['k', 's']:
            cmd_vel_mask = False
            forward   = 0.0
            leftward   = 0.0
            upward   = 0.0
            angular  = 0.0
            cmd = 'HOVER'
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            print('Hover')
        else:
            for i in range(10):
                if key == str(i):
                    cmd = formation_configs[i]
                    print_msg()
                    print(cmd)
                    cmd_vel_mask = True
            if (key == '\x03'):
                break

        if forward > MAX_LINEAR:
            forward = MAX_LINEAR
        elif forward < -MAX_LINEAR:
            forward = -MAX_LINEAR
        if leftward > MAX_LINEAR:
            leftward = MAX_LINEAR
        elif leftward < -MAX_LINEAR:
            leftward = -MAX_LINEAR
        if upward > MAX_LINEAR:
            upward = MAX_LINEAR
        elif upward < -MAX_LINEAR:
            upward = -MAX_LINEAR
        if angular > MAX_ANG_VEL:
            angular = MAX_ANG_VEL
        elif angular < -MAX_ANG_VEL:
            angular = - MAX_ANG_VEL
            
        twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward
        twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular

        for i in range(1):
            if ctrl_leader:
                leader_cmd_vel_flu_pub.publish(twist)
                leader_cmd_pub.publish(cmd)
            else:
                if not cmd_vel_mask:
                    multi_cmd_vel_flu_pub[i].publish(twist)
                multi_cmd_pub[i].publish(cmd)
                
        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

