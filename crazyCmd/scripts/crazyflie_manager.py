#! /usr/bin/env python3
# ROS modules
import time

import rospy

# Crazyflie basic modules:
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Crazyflie command modules:
from cflib.positioning.motion_commander import MotionCommander

# Crazzyflie logging modules:
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# Custom modules
from crazy_common_py.dataTypes import CfAgent
from crazy_common_py.constants import *

# MESSAGES
from crazyflie_messages.msg import RollPitchYaw

class CrazyflieManager:
    #=======================================================================================================
    #
    #                                           C O N S T R U C T O R
    #
    #=======================================================================================================
    def __init__(self, cf_agent):
        #+++++++++++++++++++++++++++++++++++++++++++++
        #        B A S I C  P R O P E R T I E S
        #+++++++++++++++++++++++++++++++++++++++++++++
        # Crazyflie ID:
        self.URI = cf_agent.URI
        self.name = cf_agent.name

        self.decks = cf_agent.decks
        self.decks_checks = []

        self.log_items = cf_agent.log_items

        # Safe stop check:
        self.safe_stop = False

        # +++++++++++++++++++++++++++++++++++++++++++++
        #              P U B L I S E R S
        # +++++++++++++++++++++++++++++++++++++++++++++
        self.attitude_publisher = rospy.Publisher('/' + self.name + '_attitude', RollPitchYaw, queue_size=1)

        # +++++++++++++++++++++++++++++++++++++++++++++
        #                M E S S A G E S
        # +++++++++++++++++++++++++++++++++++++++++++++
        self.__attitude_msg = RollPitchYaw()

        # +++++++++++++++++++++++++++++++++++++++++++++
        #              S E R V I C E S
        # +++++++++++++++++++++++++++++++++++++++++++++

        # +++++++++++++++++++++++++++++++++++++++++++++
        #                A C T I O N S
        # +++++++++++++++++++++++++++++++++++++++++++++

        # Check if we are dealing with a real crazyflie or a simulated one
        if "simulation" not in self.URI:
            self.isSimulated = False
        else:
            self.isSimulated = True

        if self.isSimulated:
            self.__simulation_init()
        else:
            self.__real_init()

    # =======================================================================================================
    #
    #                                        D E C K  C H E C K  M E T H O D S
    #
    # These methods are aimed to check if all the requested decks are properly attached to the real crazyflie
    # before instantiate it and being able to control it.
    # ========================================================================================================
    def __deckCheckCB(self, name, value_str):
        value = int(value_str)
        if value == 1:
            self.decks_checks.append(1)
            rospy.logdebug("Deck %s correctly attached to Crazyflie %s with URI %s",
                self.decks[self.__actual_deck].name, self.name, self.URI)

        else:
            print("NON OK")
            self.decks_checks.append(0)
            rospy.logerr("[ERROR] Deck %s not found on Crazyflie %s with URI %s",
                  self.decks[self.__actual_deck].name, self.name, self.URI)

    def __checkDecks(self):
        self.__actual_deck = 0
        for deck in self.decks:
            with SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache')) as scf:
                scf.cf.param.add_update_callback(group="deck", name=deck.name, cb=self.__deckCheckCB)
                time.sleep(DELAY_DECK_CHECK)
            self.__actual_deck += 1
        if 0 in self.decks_checks:
            self.all_decks_attached = False
        else:
            self.all_decks_attached = True

    # ========================================================================================================
    #
    #                                        S I M U L A T I O N  I N I T
    #
    # ========================================================================================================
    def __simulation_init(self):
        pass
    # ========================================================================================================
    #
    #                                        R E A L  C R A Z Y F L I E  I N I T
    #
    # ========================================================================================================
    def __real_init(self):
        # Performing a check of the decks:
        self.all_decks_attached = False
        self.__checkDecks()




        if self.all_decks_attached:
            with SyncCrazyflie(self.URI) as self.scf:
                with MotionCommander(self.scf, default_height=DEFAULT_TAKEOFF_HEIGHT) as self.mc:
                    rospy.logdebug("Crazyflie %s is setting up logging", self.name)
                    # Init the logconf:
                    self.__attitude_log_async()
                    self.__attitude_logconf.start()

                    rospy.logdebug("Crazyflie %s is taking off...", self.name)
                    cont = 0
                    while True:
                        cont += 1
                        if cont > 3:
                            break
                        time.sleep(1)
                    rospy.logdebug("Crazyflie %s is starting the task...", self.name)
                    self.square_motion_cw(0.2, 0.2, 90, 0.3, 0.1)
                    rospy.logdebug("Crazyflie %s has finished the task!", self.name)
                    rospy.logdebug("Crazyflie %s is landing...", self.name)
                    self.__attitude_logconf.stop()

    def exiting_operations(self):
        rospy.logdebug("Crazyflie ")
        self.mc.land()
        self.scf.close_link()
        self.safe_stop = True
    # =======================================================================================================
    #
    #                                     P U B L I S H E R S  M E T H O D S
    #
    # ========================================================================================================
    def __attitude_log_async(self):
        self.__init_logconf()
        self.scf.cf.log.add_config(self.__attitude_logconf)
        self.__attitude_logconf.data_received_cb.add_callback(self.__attitude_log_cb)


    def __attitude_log_cb(self, timestamp, data, logconf):
        self.__attitude_msg.roll = data['stabilizer.roll']
        self.__attitude_msg.pitch = data['stabilizer.pitch']
        self.__attitude_msg.yaw = data['stabilizer.yaw']
        self.attitude_publisher.publish(self.__attitude_msg)

    def __init_logconf(self):
        self.__attitude_logconf = LogConfig('stabilizer', period_in_ms=10)
        for log_item in self.log_items:
            self.__attitude_logconf.add_variable(log_item.variable_path, log_item.variable_type)

    # =======================================================================================================
    #
    #                                        T R A J E C T O R Y
    #
    # ========================================================================================================
    def square_motion_cw(self, verticalSpeed, horizontalSpeed, rotationalSpeed, squareSide, height):
        self.mc.up(height, velocity=verticalSpeed)
        for ii in range(0, 4):
            self.mc.forward(squareSide, velocity=horizontalSpeed)
            self.mc.turn_right(90, rate=rotationalSpeed)
        self.mc.down(height, velocity=verticalSpeed)




    # =======================================================================================================
    #
    #                                        XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    #
    # ========================================================================================================

    def takeoff(self):
        pass

    def land(self):
        pass

    def up(self):
        pass

    def down(self):
        pass

    def forward(self):
        pass

    def backward(self):
        pass

    def rotate(self):
        pass

def shutdown_operations():
    global cf1
    if not cf1.safe_stop:
        #cf1.exiting_operations()
        print()



if __name__ == '__main__':
    global cf1
    # Node initialization:
    rospy.init_node('cf_single_test', log_level=rospy.DEBUG)

    # Data of agent crazyflie:
    agent = CfAgent('radio://0/80/2M/E7E7E7E7E7', 'cf1')
    agent.add_deck("bcFlow2")
    agent.add_deck("bcZRanger2")
    agent.add_log_item('stabilizer.roll', 'float')
    agent.add_log_item('stabilizer.pitch', 'float')
    agent.add_log_item('stabilizer.yaw', 'float')

    # Driver initialization:
    cflib.crtp.init_drivers()

    # CrazyflieManager instantiation:
    cf1 = CrazyflieManager(agent)


    rospy.on_shutdown(shutdown_operations)

    rospy.spin()