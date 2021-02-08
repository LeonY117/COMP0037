import numpy as np

from robot_enums import Action
from robot_enums import State

# Generate the random policy
def uniformly_random_policy(environment):
    return np.ones([environment.nS, environment.nA]) / environment.nA


def tuned_random_policy(environment):
    policy = np.zeros([environment.nS, environment.nA])
    # Policy when state is high
    policy[State.HIGH, Action.WAIT] = 0.2
    policy[State.HIGH, Action.SEARCH] = 0.8
    
    # Policy when state is low
    policy[State.LOW, Action.WAIT] = 0.3
    policy[State.LOW, Action.SEARCH] = 0.6
    policy[State.LOW, Action.RECHARGE] = 0.1

    return policy
    
 
