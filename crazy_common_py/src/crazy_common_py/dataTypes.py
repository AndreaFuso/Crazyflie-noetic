import rospy

# Deck class is used to bring information of a single deck attached to the Crazyflie, informations about
# a certain deck are used in order to perform a safety check when trying to instantiate a crazyflie
class Deck():
    def __init__(self, name_):
        self.name = name_

# CfAgent is used as a "struct" bringing information of the crazyflie we want to spawn
class CfAgent():
    def __init__(self, uri_, name_="cf"):
        self.URI = uri_
        self.name = name_
        self.decks = []

    def add_deck(self,name_):
        deck = Deck(name_)
        self.decks.append(deck)
