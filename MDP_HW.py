import numpy as np

class MDPEnvironment:
    """
    MDP environment class for a simple grid world with walls, obstacles, and terminal states.
    """
    def __init__(self):
        #initialize grid representation
        self.grid_size = 5
        self.obstacles = {(2,5), (3,2), (4,5)} 
        self.walls = {(1,4):[(1,3)], \
                      (1,3): [(1,2), (1,4)], \
                      (5,4):[(5,3)], \
                      (5,3): [(5,2), (5,4)],\
                      (1,2):[(1,3)], \
                      (5,2):[(5,3)] \
                      }
        self.terminal_states = {(5,5): 100, (3,4): -1000}
        # 1: move one cell forward
        # 2: move two cells forward in direction facing
        # 3: turn right
        # 4: turn left
        self.actions = {1: -1.0, 2: -1.5, 3: -0.5, 4: -0.5} #action and cost
        self.states = []

        #fill in list of possible states
        for x in range (1, self.grid_size + 1):
            for y in range(1, self.grid_size + 1):
                if (x,y) not in self.obstacles:
                    for direction in range(1,5): #up. down, left, right
                        self.states.append((x,y,direction))

    def is_valid_state(self, state):
        """
        Check if the state is valid 
        """
        x, y, d = state
        #bounds check
        if x < 1 or x > self.grid_size or y < 1 or y > self.grid_size:
            return False
        # obstacle check
        if (x,y) in self.obstacles:
            return False
        # wall check 
        if (x,y) in self.walls:
            for wall in self.walls[(x,y)]:
                if d == 1 and wall == (x, y+1):
                    return False
                if d == 2 and wall == (x, y-1): 
                    return False
                if d == 3 and wall == (x-1, y):
                    return False
                if d == 4 and wall == (x+1, y): 
                    return False
        return True
                        
    
    def transition(self, state, action, noise=0.1):
        if tuple(state[:2]) in self.terminal_states:
            return state, 0
        x, y, d = state
        intended_action = action
        # Apply noise to action
        if np.random.rand() < noise:
            action = np.random.choice(list(self.actions.keys()))
        #next state given actions
        # move one cell forward
        if action == 1:
            if d == 1:  #up
                next_state = (x, y+1, d)
            if d == 2:  #down
                next_state = (x, y-1, d)
            if d == 3:  #left
                next_state = (x-1, y, d)
            if d == 4:  #right
                next_state = (x+1, y, d)
        # move two cells forward
        elif action == 2:
            if d == 1: 
                next_state = (x, y+2, d)
            if d == 2:
                next_state = (x, y-2, d)
            if d == 3:
                next_state = (x-2, y, d)
            if d == 4:
                next_state = (x+2, y, d)
        # turn right
        elif action == 3:
            if d == 1:
                next_state = (x, y, 4)
            elif d == 2:
                next_state = (x, y, 3)
            elif d == 3:
                next_state = (x, y, 1)
            elif d == 4:
                next_state = (x, y, 2)
        # turn left
        elif action == 4:
            if d == 1:
                next_state = (x, y, 3)
            elif d == 2:
                next_state = (x, y, 4)
            elif d == 3:
                next_state = (x, y, 2)
            elif d == 4:
                next_state = (x, y, 1)


def ValueIteration(mdpENV, gamma, noise, iterations):
    """
    Value Iteration algorithm for solving MDPs
    based on iterations not convergence
    """
    V = {state: 0 for state in mdpENV.states}  # Initialize value function
    policy = {state: None for state in mdpENV.states}  # Initialize policy
    for it in range(iterations):
        V_new = V.copy()
        for state in mdpENV.states:
            if tuple(state[:2]) in mdpENV.terminal_states:
                continue  # Skip terminal states
            max_value = float('-inf')
            best_action = None
            for action in mdpENV.actions:
                next_state, reward = mdpENV.transition(state, action)
                value = reward + gamma * V[next_state]
                if value > max_value:
                    max_value = value
                    best_action = action
            V_new[state] = max_value
            policy[state] = best_action
        # Check for convergence
        if all(abs(V_new[s] - V[s]) < noise for s in mdpENV.states):
            break
        V = V_new

    return V, policy


def main():
    mdpENV = MDPEnvironment()
    gamma = 1
    noise = 0
    iterations = 100
    V, policy = ValueIteration(mdpENV, gamma, noise, iterations)
    for state in sorted(mdpENV.states):
        print(f"State: {state}, Value: {V[state]:.2f}, Policy: {policy[state]}")
main()