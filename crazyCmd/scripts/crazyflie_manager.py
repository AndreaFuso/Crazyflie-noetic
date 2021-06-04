#! /usr/bin/env python3
# ROS modules
import time

import rospy

# Crazyflie modules
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from crazy_common_py.dataTypes import CfAgent
from crazy_common_py.constants import *

class CrazyflieManager:
    #=======================================================================================================
    #
    #                                           C O N S T R U C T O R
    #
    #=======================================================================================================
    def __init__(self, cf_agent):
        # Crazyflie ID:
        self.URI = cf_agent.URI
        self.name = cf_agent.name
        self.decks = cf_agent.decks
        self.decks_checks = []
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
                self.decks[-1].name, self.name, self.URI)

        else:
            print("NON OK")
            self.decks_checks.append(0)
            rospy.logerr("[ERROR] Deck %s not found on Crazyflie %s with URI %s",
                  self.decks[-1].name, self.name, self.URI)

    def __checkDecks(self):
        for deck in self.decks:
            with SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache')) as scf:
                scf.cf.param.add_update_callback(group="deck", name=deck.name, cb=self.__deckCheckCB)
                time.sleep(DELAY_DECK_CHECK)
        if 0 in self.decks_checks:
            self.all_decks_attached = False
        else:
            self.all_decks_attached = True

    # =======================================================================================================
    #
    #                                        S I M U L A T I O N  I N I T
    #
    # ========================================================================================================
    def __simulation_init(self):
        pass
    # =======================================================================================================
    #
    #                                        R E A L  C R A Z Y F L I E  I N I T
    #
    # ========================================================================================================
    def __real_init(self):
        # Performing a check of the decks:
        self.all_decks_attached = False
        self.__checkDecks()

        if self.all_decks_attached:
            with SyncCrazyflie(self.URI) as scf:
                with MotionCommander(scf, default_height=DEFAULT_TAKEOFF_HEIGHT) as mc:
                    rospy.logdebug("Crazyflie %s is taking off...", self.name)
                    time.sleep(2)

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

URI = 'radio://0/80/2M/E7E7E7E7E7'

is_deck_attached = False


def param_deck_flow(name, value_str):
    value = int(value_str)
    print(value)
    global is_deck_attached
    if value:
        is_deck_attached = True
        print('Deck is attached!')
    else:
        is_deck_attached = False
        print('Deck is NOT attached!')


if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('cf_single_test', log_level=rospy.DEBUG)

    # Driver initialization:
    cflib.crtp.init_drivers()

    # Data of agent crazyflie:
    agent = CfAgent('radio://0/80/2M/E7E7E7E7E7', 'cf1')
    agent.add_deck("bcFlow2")
    agent.add_deck("bcZRanger2")

    # CrazyflieManager instantiation:
    cf1 = CrazyflieManager(agent)


    rospy.spin()