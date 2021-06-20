import rospy

# Deck class is used to bring information of a single deck attached to the Crazyflie, informations about
# a certain deck are used in order to perform a safety check when trying to instantiate a crazyflie
class Deck():
    def __init__(self, name_):
        self.name = name_

class LogItem():
    def __init__(self, variable_path_, variable_type_):
        self.variable_path = variable_path_
        self.variable_type = variable_type_

# CfAgent is used as a "struct" bringing information of the crazyflie we want to spawn
class CfAgent():
    def __init__(self, uri_, name_):
        self.URI = uri_
        self.name = name_
        self.decks = []

        self.log_items = []

    def add_deck(self,name_):
        deck = Deck(name_)
        self.decks.append(deck)

    def add_log_item(self, item_path_, item_type_):
        log_item = LogItem(item_path_, item_type_)
        self.log_items.append(log_item)


class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
