import time
import jax
import jax.numpy as np
from jax import random, grad, jit, lax
import optax
import jaxopt
jax.config.update("jax_enable_x64", True)

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
plt.rcParams.update({'font.size': 10})
# gpjax version: '0.8.2'
import gpjax as gpx
from test_jax_utils import *
from test_gp_utils import *
from test_policy import policy

dynamics_type = 'ideal'
# dynamics_type = 'noisy'
# dynamics_type = 'gp'

optimizer = 'scipy'
# optimizer = 'custom_gd'

#home_path = '/home/dasc/albus/wind_disturbance_experiment/GP/gp_advanced/'
home_path = '/Users/albusfang/Coding Projects/gp_ws/Gaussian Process/GP/gp_advanced/'
# home_path = '/home/hardik/Desktop/Research/wind_disturbance_experiment/GP/gp_advanced/'
# home_path = '/home/wind_disturbance_experiment/GP/gp_advanced/'

# trajectory_path = home_path + 'circle_figure8_fullset/'
trajectory_path = home_path + 'gp_final/'
model_path = trajectory_path + 'models/'

disturbance_path = trajectory_path + 'disturbance_new.npy'
input_path = trajectory_path + 'input_new.npy'
key = random.PRNGKey(2)
horizon = 300 #200
dt = 0.05 #0.01

# custom optimizer
iter_adam_custom = 300
custom_gd_lr_rate = 0.1
grad_clip = 1.0

# scipy optimizer
iter_adam=200 #4000

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


@jit
def predict_states_ideal(X, policy_params, key):

    states = jnp.zeros((X.shape[0], horizon+1))
    states = states.at[:,0].set( X[:,0] )
    states_ref = jnp.zeros((X.shape[0], horizon))
    control_inputs = jnp.zeros((3, horizon))
    disturbance_means = jnp.zeros((3, horizon))
    disturbance_covs = jnp.zeros((3, horizon))

    def body(h, inputs):
        t = h * dt
        states, states_ref, control_inputs = inputs
        control_input, pos_ref, vel_ref = policy( t, states[:,[h]], policy_params )         # mean_position = get_mean( states, weights )
        control_inputs = control_inputs.at[:,h].set(control_input[:,0])
        next_states, _ = get_next_states_ideal( states[:,[h]], control_input, dt )
        states = states.at[:,h+1].set( next_states[:,0] )
        states_ref = states_ref.at[:,h].set( jnp.append(pos_ref[:,0], vel_ref[:,0]) )
        return states, states_ref, control_inputs
    states, states_ref, control_inputs =  lax.fori_loop( 0, horizon, body, (states, states_ref, control_inputs) )
    return states, states_ref, control_inputs, disturbance_means, disturbance_covs


@jit
def predict_states_noisy(X, policy_params, key):

    states = jnp.zeros((X.shape[0], horizon+1))
    states = states.at[:,0].set( X[:,0] )
    states_ref = jnp.zeros((X.shape[0], horizon))
    control_inputs = jnp.zeros((3, horizon))
    disturbance_means = jnp.zeros((3, horizon))
    disturbance_covs = jnp.zeros((3, horizon))

    def body(h, inputs):
        t = h * dt
        states, states_ref, key, control_inputs, disturbance_means, disturbance_covs = inputs
        control_input, pos_ref, vel_ref = policy( t, states[:,[h]], policy_params )         # mean_position = get_mean( states, weights )
        control_inputs = control_inputs.at[:,h].set(control_input[:,0])
        next_states, next_states_cov, disturbance_mean, disturbance_cov = get_next_states_noisy_predict( states[:,[h]], control_input, dt )
        disturbance_means = disturbance_means.at[:,h].set( disturbance_mean[:,0] )
        disturbance_covs = disturbance_covs.at[:,h].set( disturbance_cov[:,0] )
        key, subkey = jax.random.split(key)
        next_states = next_states + jax.random.normal( subkey, shape=(6,1) ) * jnp.sqrt( next_states_cov )
        # next_states = next_states + jax.random.normal( subkey, shape=(6,13) ) * jnp.sqrt( next_states_cov )
        states = states.at[:,h+1].set( next_states[:,0] )
        states_ref = states_ref.at[:,h].set( jnp.append(pos_ref[:,0], vel_ref[:,0]) )
        return states, states_ref, key, control_inputs, disturbance_means, disturbance_covs
    states, states_ref, key, control_inputs, disturbance_means, disturbance_covs =  lax.fori_loop( 0, horizon, body, (states, states_ref, key, control_inputs, disturbance_means, disturbance_covs) )
    return states, states_ref, control_inputs, disturbance_means, disturbance_covs

# @jit
def setup_predict_states_gp(file_path1, file_path2, file_path3, gp_train_x, gp_train_y): #, dt):

    gp0 = initialize_gp_prediction( file_path1 ) #, gp_train_x, gp_train_y[:,0].reshape(-1,1) )
    gp1 = initialize_gp_prediction( file_path2 ) #, gp_train_x, gp_train_y[:,1].reshape(-1,1) )
    gp2 = initialize_gp_prediction( file_path3 ) #, gp_train_x, gp_train_y[:,2].reshape(-1,1) )

    x = gp_train_x
    y = gp_train_y

    ###### precomputes all necessary inverses to save time ######
    D0 = gpx.Dataset(X=x, y=y[0].reshape(-1,1))
    D1 = gpx.Dataset(X=x, y=y[1].reshape(-1,1))
    D2 = gpx.Dataset(X=x, y=y[2].reshape(-1,1))
    sigma0 = gp0.compute_sigma_inv(train_data=D0)
    sigma1 = gp1.compute_sigma_inv(train_data=D1)
    sigma2 = gp2.compute_sigma_inv(train_data=D2)

    @jit
    def predict_states(X, policy_params, key):
        states = jnp.zeros((X.shape[0], horizon+1))
        states = states.at[:,0].set( X[:,0] )
        states_ref = jnp.zeros((X.shape[0], horizon))
        control_inputs = jnp.zeros((3, horizon))
        disturbance_means = jnp.zeros((3, horizon))
        disturbance_covs = jnp.zeros((3, horizon))
        def body(h, inputs):
            t = h * dt
            states, states_ref, key, control_inputs, disturbance_means, disturbance_covs = inputs
            control_input, pos_ref, vel_ref = policy( t, states[:,[h]], policy_params )         # mean_position = get_mean( states, weights )
            control_inputs = control_inputs.at[:,h].set(control_input[:,0])
            next_states, next_states_cov, disturbance_mean, disturbance_cov = get_next_states_with_gp_sigma_inv_predict( states[:,[h]], control_input, dt, [gp0, gp1, gp2],[sigma0, sigma1, sigma2], gp_train_x, gp_train_y )
            disturbance_means = disturbance_means.at[:,h].set( disturbance_mean[:,0] )
            disturbance_covs = disturbance_covs.at[:,h].set( disturbance_cov[:,0] )
            key, subkey = jax.random.split(key)
            next_states = next_states + jax.random.normal( subkey, shape=(6,1) ) * jnp.sqrt( next_states_cov )
            states = states.at[:,h+1].set( next_states[:,0] )
            states_ref = states_ref.at[:,h].set( jnp.append(pos_ref[:,0], vel_ref[:,0]) )
            return states, states_ref, key, control_inputs, disturbance_means, disturbance_covs
        states, states_ref, key, control_inputs, disturbance_means, disturbance_covs =  lax.fori_loop( 0, horizon, body, (states, states_ref, key, control_inputs, disturbance_means, disturbance_covs) )
        return states, states_ref, control_inputs, disturbance_means, disturbance_covs
    
    return predict_states


def setup_future_reward_func(file_path1, file_path2, file_path3, dynamics_type='ideal'):

    gp0 = initialize_gp_prediction( file_path1 ) #, gp_train_x, gp_train_y[:,0].reshape(-1,1) )
    gp1 = initialize_gp_prediction( file_path2 ) #, gp_train_x, gp_train_y[:,1].reshape(-1,1) )
    gp2 = initialize_gp_prediction( file_path3 ) #, gp_train_x, gp_train_y[:,2].reshape(-1,1) )

    x = gp_train_x
    y = gp_train_y

    ###### precomputes all necessary inverses to save time ######
    D0 = gpx.Dataset(X=x, y=y[0].reshape(-1,1))
    D1 = gpx.Dataset(X=x, y=y[1].reshape(-1,1))
    D2 = gpx.Dataset(X=x, y=y[2].reshape(-1,1))
    sigma0 = gp0.compute_sigma_inv(train_data=D0)
    sigma1 = gp1.compute_sigma_inv(train_data=D1)
    sigma2 = gp2.compute_sigma_inv(train_data=D2)

    @jit
    def compute_reward(X, policy_params, gp_train_x, gp_train_y):
        '''
        Performs Gradient Descent
        '''
        states, weights = initialize_sigma_points(X)
        kx = policy_params[0]
        kv = policy_params[1]
        w1 = 0.5
        w2 = 0.1
        # reward = 0 + w1 * (kx-7)**2 + w2 * (kv-4)**2
        reward = 0 + w1 * (kx)**2 + w2 * (kv)**2
        # reward = 0
        def body(h, inputs):
            '''
            Performs UT-EC with 6 states
            '''
            t = h * dt
            reward, states, weights = inputs
            control_inputs, pos_ref, vel_ref = policy( t, states, policy_params )         # mean_position = get_mean( states, weights )
            if dynamics_type=='ideal':
                next_states_mean, next_states_cov = get_next_states_ideal( states, control_inputs, dt )
            elif dynamics_type=='noisy':
                next_states_mean, next_states_cov = get_next_states_noisy( states, control_inputs, dt )
            elif dynamics_type=='gp':
                next_states_mean, next_states_cov = get_next_states_with_gp_sigma_inv( states, control_inputs, dt, [gp0, gp1, gp2], [sigma0, sigma1, sigma2], gp_train_x, gp_train_y )
            next_states_expanded, next_weights_expanded = sigma_point_expand_with_mean_cov( next_states_mean, next_states_cov, weights)
            next_states, next_weights = sigma_point_compress( next_states_expanded, next_weights_expanded )
            states = next_states
            weights = next_weights
            reward = reward + reward_func( states, weights, pos_ref, vel_ref ) # reward is loss
            return reward, states, weights
        reward =  lax.fori_loop( 0, horizon, body, (reward, states, weights) )[0]
        return reward
    return compute_reward

print(model_path)
file_path1 = model_path + "gp_model_x_norm5_clipped.pkl"
file_path2 = model_path + "gp_model_y_norm5_clipped.pkl"
file_path3 = model_path + "gp_model_z_norm5_clipped.pkl"


gp_train_x = jnp.load(input_path)
# gp_train_x = gp_train_x[::80]
gp_train_x = gp_train_x[::140]
gp_train_y = jnp.load(disturbance_path)
gp_train_y = gp_train_y[::140].T
t0 = time.time()

def train_policy_jaxscipy(init_state, params_policy, gp_train_x, gp_train_y):
    minimize_func = lambda params: get_future_reward( init_state, params, gp_train_x, gp_train_y )
    solver = jaxopt.ScipyMinimize(fun=minimize_func, maxiter=iter_adam)
    params_policy, cost_state = solver.run(params_policy)
    return params_policy

def train_policy_custom(init_state, params_policy, gp_train_x, gp_train_y):

    def body(i, inputs):
        params_policy = inputs
        params_policy_grad = get_future_reward_grad( init_state, params_policy, gp_train_x, gp_train_y )
        params_policy_grad = jnp.clip( params_policy_grad, -grad_clip, grad_clip )
        params_policy = params_policy - custom_gd_lr_rate * params_policy_grad
        return params_policy
    params_policy = lax.fori_loop(0, iter_adam_custom, body, params_policy)


    # for i in range(iter_adam_custom):
    #     params_policy_grad = get_future_reward_grad( init_state, params_policy, gp_train_x, gp_train_y )
    #     params_policy_grad = jnp.clip( params_policy_grad, -grad_clip, grad_clip )
    #     params_policy = params_policy - custom_gd_lr_rate * params_policy_grad
    return params_policy
    
# def generate_state_vector(key, n):
#     return jax.random.normal(key, (n, 1))
def generate_state_vector(key, n):
    state_vector = jax.random.normal(key, (n, 1))
    state_vector = state_vector.at[0,0].set(0.0)
    state_vector = state_vector.at[1,0].set(0.0)
    return state_vector

# Example usage:
key = jax.random.PRNGKey(0)  # Initialize the random key
n = 6  # Size of the state vector
state_vector = generate_state_vector(key, n)
print(state_vector)

def setup_predict_states(file_path1, file_path2, file_path3, dynamics_type='ideal'):
    if dynamics_type=='ideal':
        return predict_states_ideal
    elif dynamics_type=='noisy':
        return predict_states_noisy
    elif dynamics_type=='gp':
        predict_states_gp = setup_predict_states_gp(file_path1, file_path2, file_path3, gp_train_x, gp_train_y)
        return predict_states_gp

# first run
get_future_reward = setup_future_reward_func(file_path1, file_path2, file_path3, dynamics_type=dynamics_type)
predict_states = setup_predict_states(file_path1, file_path2, file_path3, dynamics_type=dynamics_type)
get_future_reward_grad = jit(grad(get_future_reward, argnums=1))
# get_future_reward( state_vector, jnp.array([14.0, 7.4]), gp_train_x, gp_train_y )
# get_future_reward_grad( state_vector, jnp.array([14.0, 7.4]), gp_train_x, gp_train_y )
get_future_reward( state_vector, jnp.array([7.0, 4.0]), gp_train_x, gp_train_y )
get_future_reward_grad( state_vector, jnp.array([7.0, 4.0]), gp_train_x, gp_train_y )
# plot trajectory with initial parameter
# params_init = jnp.array([14.0, 7.4])
params_init = jnp.array([7.0, 4.0])
key, subkey = jax.random.split(key)
states, states_ref, control_inputs, disturbance_means, disturbance_covs = predict_states(state_vector, params_init, subkey)
key, subkey = jax.random.split(key)
states2, states_ref2, control_inputs2, disturbance_means2, disturbance_covs2 = predict_states(state_vector, params_init, subkey)
fig, ax = plt.subplots()
ax.plot(states_ref[0,:], states_ref[1,:], 'r', label='reference')
ax.plot(states[0,:], states[1,:], 'g', label='states unoptimized')
ax.plot(states2[0,:], states2[1,:], 'g--', label='states2 unoptimized')
# ax.set_xlim([-0.4,1.3])
# ax.set_ylim([-1.3, 0.4])

ax.set_xlabel('X')
ax.set_ylabel('Y')

fig_acc, ax_acc = plt.subplots(3)
ax_acc[0].plot( control_inputs[0,:], 'g', label='Control acceleration' )
ax_acc[1].plot( control_inputs[1,:], 'g' )
ax_acc[2].plot( control_inputs[2,:], 'g' )
ax_acc[0].plot( disturbance_means[0,:], 'r', label='Disturbance' )
ax_acc[1].plot( disturbance_means[1,:], 'r' )
ax_acc[2].plot( disturbance_means[2,:], 'r' )
ax_acc[0].plot( control_inputs2[0,:], 'g--')
ax_acc[1].plot( control_inputs2[1,:], 'g--' )
ax_acc[2].plot( control_inputs2[2,:], 'g--' )
ax_acc[0].plot( disturbance_means2[0,:], 'r--')
ax_acc[1].plot( disturbance_means2[1,:], 'r--' )
ax_acc[2].plot( disturbance_means2[2,:], 'r--' )
ax_acc[0].set_ylabel('X')
ax_acc[1].set_ylabel('Y')
ax_acc[2].set_ylabel('Z')
# w1 = 0.2
# w2 = 0.05
# plt. savefig(f'gain_tuning/plots/trajectory_wx{w1}_wv{w2}.png')
# plt.show()

# params_optimized = train_policy_jaxscipy(state_vector, params_init, gp_train_x, gp_train_y)
if optimizer=='scipy':
    params_optimized = train_policy_jaxscipy(state_vector, params_init, gp_train_x, gp_train_y)
elif optimizer=='custom_gd':
    params_optimized = train_policy_custom(state_vector, params_init, gp_train_x, gp_train_y)
else:
    print(f"NOT IMPLEMENTED ERROR")
    exit()
print(f"new params: {params_optimized}")
ax.set_title(f'gains: {params_optimized[0]},{params_optimized[1]}')
key, subkey = jax.random.split(key)
states_optimized, states_ref_optimized, control_inputs_optimized, disturbance_means_optimized, disturbance_covs_optimized = predict_states(state_vector, params_optimized, subkey)
key, subkey = jax.random.split(key)
states_optimized2, states_ref_optimized2, control_inputs_optimized2, disturbance_means_optimized2, disturbance_covs_optimized2 = predict_states(state_vector, params_optimized, subkey)
ax.plot(states_optimized[0,:], states_optimized[1,:], 'k', label='states optimized')
ax.plot(states_optimized2[0,:], states_optimized2[1,:], 'k--', label='states2 optimized')

ax_acc[0].plot( control_inputs_optimized[0,:], 'b', label='Control acceleration optimized' )
ax_acc[1].plot( control_inputs_optimized[1,:], 'b' )
ax_acc[2].plot( control_inputs_optimized[2,:], 'b' )
ax_acc[0].plot( disturbance_means_optimized[0,:], 'k', label='Disturbance optimized' )
ax_acc[1].plot( disturbance_means_optimized[1,:], 'k' )
ax_acc[2].plot( disturbance_means_optimized[2,:], 'k' )
ax_acc[0].plot( control_inputs_optimized2[0,:], 'b--', label='Control acceleration optimized' )
ax_acc[1].plot( control_inputs_optimized2[1,:], 'b--' )
ax_acc[2].plot( control_inputs_optimized2[2,:], 'b--' )
ax_acc[0].plot( disturbance_means_optimized2[0,:], 'k--', label='Disturbance optimized' )
ax_acc[1].plot( disturbance_means_optimized2[1,:], 'k--' )
ax_acc[2].plot( disturbance_means_optimized2[2,:], 'k--' )

ax.legend()
ax_acc[0].legend()
ax_acc[1].legend()
ax_acc[2].legend()
# plt.savefig(f"gain_tuning/plots/component_wx{w1}_wv{w2}.png")
plt.show()





