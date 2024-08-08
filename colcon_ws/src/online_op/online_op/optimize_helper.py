import jax.numpy as jnp
import numpy as np
from test_jax_utils import *
def initialize_sigma_points(X):
        '''
        Returns Equally weighted Sigma Particles
        '''
        n = X.shape[0]
        num_points = 2*n + 1
        X = X.reshape(-1,1)
        sigma_points = np.repeat( X, num_points, axis=1 )
        weights = np.ones((1,num_points)) * 1.0/( num_points )
        return sigma_points, weights
    
def reward_func(states, weights, pos_ref, vel_ref):
    '''
    calculates mean squared error
    inputs: states and the weights of sigma points
    returns: calculated reward
    '''
    ex = states[0:3] - pos_ref
    ev = states[3:6] - vel_ref

    ex_ev_mean = get_mean(jnp.append(ex, ev, axis=0), weights )

    pos_factor = 1.0
    vel_factor = 0.1
    reward = pos_factor * jnp.sum(ex_ev_mean[0:3] ** 2) + vel_factor * jnp.sum(ex_ev_mean[3:6] ** 2)
    return reward
