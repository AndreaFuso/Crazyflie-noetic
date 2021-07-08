#! /usr/bin/env python3
import rospy
import sys
from curtsies import Input
#import keyboard
from crazy_common_py.dataTypes import Vector3
from crazyflie_messages.msg import KeyboardKeys

class Commands:
    def __init__(self):
        self.W = False
        self.S = False
        self.A = False
        self.D = False

        self.Q = False
        self.E = False

        self.up = False
        self.down = False

def FillMessage(commands=Commands()):
    output = KeyboardKeys()
    output.W = commands.W
    output.S = commands.S
    output.A = commands.A
    output.D = commands.D
    output.Q = commands.Q
    output.E = commands.E
    output.up = commands.up
    output.down = commands.down

    return output

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('keyboard_control', log_level=rospy.INFO)

    key_pub = rospy.Publisher('/keys', KeyboardKeys, queue_size=1)

    rate = rospy.Rate(500)
    print('Commands to control the drone:')
    print('\tA/D: left/right\n\tW/S: forward/backward\n\tQ/E: yaw rotation\n\tupArrow/downArrow: thrust\nP TO EXIT')

    keysPressed_msg = KeyboardKeys()
    keysPressed = Commands()

    while not rospy.is_shutdown():
        #if keyboard.is_pressed('q'):
            #print('PRESSED Q')
        '''with Input(keynames='curtsies') as input_generator:
            for e in input_generator:
                key = str(e)
                if key == 'w':
                    keysPressed.W = True
                else:
                    keysPressed.W = False

                if key == 's':
                    keysPressed.S = True
                else:
                    keysPressed.S = False

                if key == 'a':
                    keysPressed.A = True
                else:
                    keysPressed.A = False

                if key == 'd':
                    keysPressed.D = True
                else:
                    keysPressed.D = False

                if key == 'q':
                    keysPressed.Q = True
                else:
                    keysPressed.Q = False

                if key == 'e':
                    keysPressed.E = True
                else:
                    keysPressed.E = False

                if key == '<UP>':
                    keysPressed.up = True
                else:
                    keysPressed.up = False

                if key == '<DOWN>':
                    keysPressed.down = True
                else:
                    keysPressed.down = False

                if key == 'p':
                    sys.exit()
                input_generator.send(timeout=)
                keysPressed_msg = FillMessage(keysPressed)
                key_pub.publish(keysPressed_msg)'''




    rospy.spin()
