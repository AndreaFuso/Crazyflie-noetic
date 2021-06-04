#! /usr/bin/env python3
# ROS modules
import rospy

# Crazyflie modules
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from crazy_common_py.dataTypes import CfAgent
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
        if value:
            self.decks_checks[self.__actualDeck] = True
            rospy.logdebug("Deck %s correctly attached to Crazyflie %s with URI %s",
                           self.decks_checks[self.__actualDeck].name, self.name, self.URI)
        else:
            self.decks_checks[self.__actualDeck] = False
            rospy.logerr("[ERROR] Deck %s not found on Crazyflie %s with URI %s",
                         self.decks_checks[self.__actualDeck].name, self.name, self.URI)

    def __checkDecks(self):
        self.decks_checks = []
        self.__actualDeck = 0
        for deck in self.decks:
            with SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache')) as scf:
                scf.cf.param.add_update_callback(group=deck.group, name=deck.name, cb=self.__deckCheckCB)
            self.__actualDeck += 1
        if False in self.decks_checks:
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



