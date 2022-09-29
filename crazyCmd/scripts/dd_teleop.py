#! /usr/bin/env python3
import rospy
import sys, select
import tty, termios
import actionlib
from crazy_common_py.dataTypes import Vector3
from crazyflie_messages.msg import Position
from crazy_common_py.default_topics import DEFAULT_TAKEOFF_ACT_TOPIC
from crazyflie_messages.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback

msg_init = """
Control the Crazyflie with the keyboard!
---------------------------
Moving around:
    q    w    e
    a    s    d
    z    x    c


space key : force stop

CTRL-C to quit
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

if __name__ == '__main__':

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_crazy')
    VELOCITY = 0.3

    take_off_actn = actionlib.SimpleActionClient('/cf1/' + DEFAULT_TAKEOFF_ACT_TOPIC, TakeoffAction)
    take_off_actn.wait_for_server()

    goal = TakeoffGoal()
    res = TakeoffResult
    goal.takeoff_height = 0.5
    take_off_actn.send_goal(goal)
    # take_off_actn.wait_for_result(res)

    cmd_vel_pub = rospy.Publisher('/cf1/cmd_vel', Position, queue_size=1)
    cmd_vel = Position()

    while cmd_vel_pub.get_num_connections() == 0:
        rospy.sleep(1)

    des_vel = Vector3()

    try:
        rospy.loginfo(msg_init)
        while not rospy.is_shutdown():            
            key = getKey()
            if key == 'w':
                des_vel.x = VELOCITY; des_vel.y = 0                
            elif key == 'a':
                des_vel.x = 0; des_vel.y = VELOCITY                
            elif key == 'd':
                des_vel.x = 0; des_vel.y = -VELOCITY
            elif key == 'x':
                des_vel.x = -VELOCITY; des_vel.y = 0
            elif key == 's':
                des_vel.x = 0; des_vel.y = 0
            elif key == 'q':
                des_vel.x = VELOCITY; des_vel.y = VELOCITY
            elif key == 'e':
                des_vel.x = VELOCITY; des_vel.y = -VELOCITY
            elif key == 'c':
                des_vel.x = -VELOCITY; des_vel.y = -VELOCITY
            elif key == 'z':
                des_vel.x = -VELOCITY; des_vel.y = VELOCITY
            elif key == ' ':
                break


            cmd_vel.desired_velocity = des_vel
            cmd_vel_pub.publish(cmd_vel)        
    
    
    finally:
        final_vel = Vector3()
        cmd_vel.desired_velocity = final_vel
        cmd_vel_pub.publish(cmd_vel)
        
    rospy.spin()
