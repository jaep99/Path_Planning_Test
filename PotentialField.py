import ld06
import numpy as np
from numpy.linalg import norm

#using formulas from https://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf

attractive_const = 1
repulsive_const = 1
repulsive_limit = 5 # ignore obstacle if futher than 5 meters away
step_size = .2

def attractive_potential(position, goal):
    print(position)
    print(goal)
    return attractive_const*(position - goal)

def repulsive_potential(position):
    potential = 0
    for obstacle in ld06.obstacles:
        d = norm(np.array(obstacle).flatten()- position)
        if d < repulsive_limit and d>.1:
            potential += (1/repulsive_limit - 1/d)*(1/d**2)*(position - np.array(obstacle).flatten())
            
    return .01*potential
def potential_field(curr, curr_wp, goal):
    position = np.array(curr[0])
    potential = attractive_potential(position, goal) + repulsive_potential(position)
    next_wp = np.array(curr_wp[0]).flatten() - step_size*potential
    return (next_wp[0], next_wp[1])