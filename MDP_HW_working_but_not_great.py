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
                        
    
    def transition(self, state, action):
        if tuple(state[:2]) in self.terminal_states:
            return state, 0
        x, y, d = state

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
        if action == 2:
            if d == 1: 
                next_state = (x, y+2, d)
            if d == 2: 
                next_state = (x, y-2, d)
            if d == 3: 
                next_state = (x-2, y, d)
            if d == 4: 
                next_state = (x+2, y, d)
        # turn right
        if action == 3:
            if d == 1: 
                next_state = (x, y, 4)
            if d == 2:
                next_state = (x, y, 3)
            if d == 3: 
                next_state = (x, y, 1)
            if d == 4: 
                next_state = (x, y, 2)
        if action == 4:
            # turn left
            if d == 1: 
                next_state = (x, y, 3)
            if d == 2: 
                next_state = (x, y, 4)
            if d == 3: 
                next_state = (x, y, 2)
            if d == 4: 
                next_state = (x, y, 1)
            
        #if next state valid(in bounds, not hitting a wall, not in an obstacle spot) keep it and calculate reward
        #else next_state = state and calculate reward
        reward = self.actions[action]
        if self.is_valid_state(next_state):
            if tuple(next_state[:2]) in self.terminal_states:
                reward += self.terminal_states[tuple(next_state[:2])]
            return next_state, reward
        else:
            return state, reward


def ValueIteration(mdpENV, gamma, noise, iterations):
    """
    Value Iteration algorithm with stochastic transitions (noise)
    noise = probability of taking a random adjacent action instead of the intended one.
    """
    V = {state: 0 for state in mdpENV.states}
    policy = {state: 0 for state in mdpENV.states}

    actions = list(mdpENV.actions.keys())
    for i in range(iterations):
        V_new = V.copy()
        #skip terminal state
        for state in mdpENV.states:
            if tuple(state[:2]) in mdpENV.terminal_states:
                continue  
            #track values for each action
            action_values = {}
            for action in actions:
                # Define probabilities calculate expected value based on noise
                prob_main = 1 - noise
                prob_side = noise / (len(actions) - 1)
                expected_value = 0
                for a in actions:
                    if a == action:
                        p = prob_main
                    else:
                        p = prob_side
                    next_state, reward = mdpENV.transition(state, a)
                    expected_value += p * (reward + gamma * V[next_state])
                action_values[action] = expected_value
            # retrieve policy
            best_action = max(action_values, key=action_values.get)
            V_new[state] = action_values[best_action]
            policy[state] = best_action
            if (i < 10):
                for state in sorted(mdpENV.states):
                    print(f"iteration {i+1}: ")
                    print(f"State: {state}, Value: {V[state]:.2f}, Best Action: {policy[state]}")
        V = V_new

    return V, policy


def main():
    mdpENV = MDPEnvironment()
    #part B
    V, policy = ValueIteration(mdpENV, gamma=1, noise=0, iterations=100)
main()