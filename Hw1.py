import random
from itertools import permutations

def generate_states(n, print_states=False):
    """
    Input: n representing rows and comums for nxn square game
    returns: all possible states of nxn square
    SOLVES PART A
    """
    state_0 = {i for i in range(n**2)}
    state_0_list = list(state_0)
    states = []
    #generate all 362,000 states as a list
    for p in permutations(state_0_list):
        states.append(p)
    
    if(print_states):
        print(states)
    return states

def oddneighborcheck(state):
    """
    Input: flattned 1d array representing 2D 3x3 array
    Output: True if no value is odd and has an odd neighbor
    """
    n = int(len(state)**.5)
            
    for i, val in enumerate(state) :
        if val % 2 != 0:
            #check up
            if 0 <= i-n < len(state) and (state[i-n] % 2 != 0):
                return False
            #check down
            if 0 <= i+n < len(state) and (state[i+n] % 2 != 0): 
                return False
            #check left
            if i%n != 0 and (state[i-1] % 2 != 0): 
                return False
            #check right
            if (i+1)%n != 0 and (state[i+1] % 2 != 0): 
                return False
    return True
        
            

def get10States(states):
    """
    input: list of all states.
    output: 10 randomly selected states must not have odd numbered neighbors
    SOLVES PART B
    """
    chosen_10 = []
    while len(chosen_10) != 10:
        chosen_state = random.choice(states)
        if oddneighborcheck(chosen_state):
            chosen_10.append(chosen_state)
            states.remove(chosen_state)
    return chosen_10

def puzzle_move(state, action):
    """
    Input: 1d array representing a state and an action (up, down, left, right)
    output: next state
    show resulting state after moving blank piece
    SOLVES PART C
    """
    ### you can also represent all the states as a graph
    n = int(len(state)**.5)
    idx = state.index(0)
    new_state = state.copy()

    if 0 <= idx-n < len(state) and (state[idx-n] % 2 != 0) and action == 1:
        #return the new state  
        new_state[idx] = state[idx-n]
        new_state[idx-n] = 0
    #check down
    elif 0 <= idx+n < len(state) and (state[idx+n] % 2 != 0) and action == 2: 
        new_state[idx] = state[idx+n]
        new_state[idx+n] = 0
    #check left
    elif idx%n != 0 and (state[idx-1] % 2 != 0) and action == 3: 
        new_state[idx] = state[idx-1]
        new_state[idx-1] = 0
    #check right
    elif (idx+1)%n != 0 and (state[idx+1] % 2 != 0) and action == 4: 
        new_state[idx] = state[idx+1]
        new_state[idx+1] = 0
    else:
        print("invalid input")
        return state

    return new_state

def rand_div_3(state):
    """
    Input: 1d array representing nxn grid
    SOLVES part D
    """

    n = int(len(state)**.5)
    solved = False
    steps = 0
    while not solved:
        solved = True
        for i in range(n):
            total = 0
            #check if a row is divisible by 3
            total += (100*state[i*n])
            total += (10*state[(i*n) + 1])
            total += (1*state[(i*n) + 2]) 
            if total % 3 > 0:
                solved = False
                state = puzzle_move(state, random.randint(1, 4))
                steps += 1
                break
    return state, steps

def main(): 
    states = generate_states(3)
    print(get10States(states))
    print(puzzle_move([7,2,4,5,0,6,8,3,1], 3)) #sample input from part C
    print(rand_div_3([7,2,4,5,0,6,8,3,1]))
    return 0

if __name__ == "__main__":
    main()




#debug
import random

def puzzle_move(state, action):
    """
    Input: 1d array representing a state and an action (up, down, left, right)
    Output: next state after moving blank piece
    """
    n = int(len(state)**.5)
    idx = state.index(0)
    new_state = state.copy()

    # up
    if 0 <= idx-n < len(state) and action == 1:
        new_state[idx], new_state[idx-n] = new_state[idx-n], new_state[idx]
    # down
    elif 0 <= idx+n < len(state) and action == 2: 
        new_state[idx], new_state[idx+n] = new_state[idx+n], new_state[idx]
    # left
    elif idx%n != 0 and action == 3: 
        new_state[idx], new_state[idx-1] = new_state[idx-1], new_state[idx]
    # right
    elif (idx+1)%n != 0 and action == 4: 
        new_state[idx], new_state[idx+1] = new_state[idx+1], new_state[idx]
    else:
        return state  # invalid move, return unchanged

    return new_state


def rand_div_3(state):
    """
    Randomly shuffle until each row makes a 3-digit number divisible by 3.
    """
    n = int(len(state)**.5)
    solved = False
    steps = 0
    path = [state]

    while not solved:
        solved = True
        for i in range(n):
            num = state[i*n]*100 + state[i*n+1]*10 + state[i*n+2]
            if num % 3 != 0:
                solved = False
                state = puzzle_move(state, random.randint(1, 4))
                steps += 1
                path.append(state)
                break  # restart checking after move

    return state, steps, path
