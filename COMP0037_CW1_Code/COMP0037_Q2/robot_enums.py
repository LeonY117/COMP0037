# Declare common types
from enum import IntEnum

# The actions the robot can do
class Action(IntEnum):
    SEARCH = 0
    WAIT = 1
    RECHARGE = 2

    @classmethod
    def draw_random_action(cls):
        return random.choice([Actions.SEARCH, Actions.WAIT, Actions.RECHARGE])

# The states the battery can take
class State(IntEnum):
    HIGH = 0
    LOW = 1
    FLAT = 2
