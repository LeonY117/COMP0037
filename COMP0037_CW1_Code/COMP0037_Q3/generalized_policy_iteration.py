#! /usr/bin/env python3

# Import statements
import numpy as np
from simple_grid_env import SimpleGridEnv
from clean_airport_env import CleanAirportEnv
from robot_enums import Heading
from robot_enums import Action
from plotting import ValueFunctionPlotter
from plotting import PolicyPlotter

class GeneralPolicyIteration(object):
    """
    @info: Class constructor.
    @param: Environment (clean airport or simple grid).
    """

    def __init__(self, environment, discount_factor):
        self.env = environment
        self.discount_factor = discount_factor

    """
    @info: Evaluate the given policy.
    @param: policy: state to action mapping.
            theta: Stopping condition.
            max_iteration: Maximum number of iterations before automatically returned.
    @return: V: state-value function.
    """

    def policy_evaluation(self, policy, theta=1e-9, max_iterations=1e3):
        # Initialise value function for each state as zero, V(terminal) = 0, others can be arbitrary
        V = np.zeros([self.env.grid_rows, self.env.grid_cols, 4])

        print("policy_evaluation: Insert your implementation here")
        
        return V

    """
    @info:  Improve the given policy.
    @param: policy: state to action mapping.
            V:  State-value function.
    @return: policy: Improved policy.
    """

    def policy_improvement(self, policy, V):

        stable_policy = True

        print("policy_improvement: Insert your implementation here")
        
        return policy, stable_policy

    """
    @info:  Policy iteration (using iterative policy evaluation).
            max_iteration: Maximum number of iterations before automatically returned.
    @return: policy: State to action mapping.
             V: State-value function.
    """

    def policy_iteration(self, max_iterations=1e3):
        # Initialize a random policy
        policy = np.random.randint(0, self.env.nA, size=(self.env.grid_rows, self.env.grid_cols, len(Heading)))
        V = np.zeros([self.env.grid_rows, self.env.grid_cols, len(Heading)])

        # Create functions for plotting
        value_function_plotter = ValueFunctionPlotter()
        value_function_plotter.plot(V)
        policy_plotter = PolicyPlotter()
        policy_plotter.plot(policy)

        for i in range(int(max_iterations)):

            print("policy_iteration: Insert your implementation here")

        return policy, V

    """
    @info:  Value iteration
            theta: Stopping condition.
            max_iteration: Maximum number of iterations before automatically returned.
    @return: policy: State to action mapping.
             V: State-value function.
    """

    def value_iteration(self, theta=1e-9, max_iterations=1e3):
        # Initialize a random policy
        policy = np.random.randint(0, self.env.nA, size=(self.env.grid_rows, self.env.grid_cols, len(Heading)))
        V = np.zeros([self.env.grid_rows, self.env.grid_cols, len(Heading)])

        # Create functions for plotting
        value_function_plotter = ValueFunctionPlotter()
        value_function_plotter.plot(V)
        policy_plotter = PolicyPlotter()
        policy_plotter.plot(policy)

        print("value_iteration: Insert your implementation here")

        return policy, V


if __name__ == '__main__':

    # Which type of environment?
    use_simple_grid = True

    # Which algorithms? Both can be enabled at the same time
    policy_itr = True
    value_itr = False

    if use_simple_grid:
        # Simple Grid Env
        grid_rows = 3
        grid_cols = 4
        start_state = (0, 0, Heading.NORTH)

        # Simple example
        win_states = [(0, 3)]
        lose_states = [(1, 3)]

        # Simple gridworld
        robot = SimpleGridEnv(grid_rows, grid_cols, start_state, win_states, lose_states)

    else:
        
        # Creating an object of type CleanAirportEnv
        # Locations matching the grid airport
        grid_rows = 7
        grid_cols = 9
        start_state = (0, 0, Heading.EAST)
        cleaning_locations = [(0, 5), (1, 4), (1, 5), (2, 0), (2, 1), (2, 2), (4, 0), (4, 7)]
        traffic_locations = [(1, 2), (5, 1), (5, 3), (5, 5), (5, 7)]
        trapdoor_location = [(2, 8, Heading.EAST), (4, 8, Heading.EAST)]
        customs_barrier_row = 3
        # Create object with init method requirements
        robot = CleanAirportEnv(grid_rows, grid_cols, start_state, cleaning_locations, traffic_locations,
                                trapdoor_location, customs_barrier_row)
        
    # Run the requested algorithms
    gpi = GeneralPolicyIteration(robot, discount_factor=1.0)
    if policy_itr:
        # Policy iteration
        policy, value = gpi.policy_iteration()
        input("Press Enter to continue...")
    if value_itr:
        # Value iteration
        policy, value = gpi.value_iteration()
        input("Press Enter to continue...")
