import jax.numpy as jnp
import jax
import numpy as np
from jax import grad, jit
from .test_jax_utils import *
from .test_gp_utils_sparse import get_next_states_with_gp_sigma_inv
from .test_policy import policy, circle_pos_vel_acc, figure8_pos_vel_acc
from jax.experimental import host_callback as hcb

horizon = 50
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
#     print("pos error: ", ex.item())
#     hcb.id_print(ex)
#     hcb.id_print(ev)
    ex_ev_mean = get_mean(jnp.append(ex, ev, axis=0), weights )

    pos_factor = 1.0
    vel_factor = 0.1
    reward = pos_factor * jnp.sum(ex_ev_mean[0:3] ** 2) + vel_factor * jnp.sum(ex_ev_mean[3:6] ** 2)
#     print("reward: ", reward.item())
    return reward
@jit
def get_future_reward(state, params_policy, gps, sigma_inv, gp_train_x, gp_train_y, deltaT, trajectory_type, parameters):
    print("Calculating Reward")
    print("state vector shape is: ", state.shape)
    states,weights = initialize_sigma_points( state )
    w1 = 0.5
    w2 = 0.1
    kx = params_policy[0]
    kv = params_policy[1]
    reward = w1 * (kx**2) + w2 * (kv**2)
    op_dt = 0.05
    gp0,gp1,gp2 = gps
    sigma0,sigma1,sigma2 = sigma_inv
    def body(h, inputs):
        '''
        Performs UT-EC with 6 states
        '''
        t = h * op_dt+ deltaT
        reward, states, weights = inputs
        # ref_pos, ref_vel, ref_acc = find_ref_pos_vel_acc(trajectory_type,t, trajectory_parameters)
        ref_pos,ref_vel,ref_acc = find_ref_pos_vel_acc(trajectory_type,t,parameters)
        ###### fixed policy ######
        
        control_inputs, pos_ref, vel_ref = policy( states, params_policy, [ref_pos,ref_vel,ref_acc])         # mean_position = get_mean( states, weights )
        # hcb.id_print(control_inputs)
        next_states_mean, next_states_cov = get_next_states_with_gp_sigma_inv( states, control_inputs, op_dt, [gp0, gp1, gp2], [sigma0, sigma1, sigma2], gp_train_x, gp_train_y )
        next_states_expanded, next_weights_expanded = sigma_point_expand_with_mean_cov( next_states_mean, next_states_cov, weights)
        next_states, next_weights = sigma_point_compress( next_states_expanded, next_weights_expanded )
        states = next_states
        weights = next_weights
        reward = reward + reward_func( states, weights, pos_ref, vel_ref ) # reward is loss
        return reward, states, weights
    reward =  lax.fori_loop( 0, horizon, body, (reward, states, weights) )[0]
    return reward
get_future_reward_grad = jit(grad(get_future_reward, argnums=1))

def find_ref_pos_vel_acc(trajectory_type, deltaT, parameters):
        radius, angular_vel, center_x, center_y = parameters
        # print(f"radius: {radius}, ")
        print(type(trajectory_type))
        print((trajectory_type))
        if (trajectory_type) == 0: #'circle'
            pos_vel_acc = circle_pos_vel_acc
        else:
            pos_vel_acc = figure8_pos_vel_acc
        ref_pos,ref_vel,ref_acc = pos_vel_acc(deltaT, radius, angular_vel, center_x, center_y)
        return ref_pos,ref_vel,ref_acc