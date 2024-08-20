import time
import jax
import jax.numpy as np
from jax import random, grad, jit, lax, value_and_grad, jacrev, jacfwd
import optax
import jaxopt
jax.config.update("jax_enable_x64", True)

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
plt.rcParams.update({'font.size': 10})
# gpjax version: '0.8.2'
import gpjax as gpx
from test_jax_utils import *
from test_gp_utils_sparse import *
from test_policy import policy

# dynamics_type = 'ideal'
# dynamics_type = 'noisy'
dynamics_type = 'gp'

# optimizer = 'scipy'
optimizer = 'custom_gd'

#home_path = '/home/dasc/albus/wind_disturbance_experiment/GP/gp_advanced/'
# home_path = '/Users/albusfang/Coding Projects/gp_ws/Gaussian Process/GP/gp_advanced/'
# home_path = '/home/hardik/Desktop/Research/wind_disturbance_experiment/GP/gp_advanced/'
home_path = '/home/wind_disturbance_experiment/GP/gp_advanced/'

# trajectory_path = home_path + 'circle_figure8_fullset/'
trajectory_path = home_path + 'gp_final/'
model_path = trajectory_path + 'models/'

disturbance_path = trajectory_path + 'disturbance_new.npy'
input_path = trajectory_path + 'input_new.npy'
key = random.PRNGKey(2)
horizon = 30 #50 #300 #200
simT = 300 #300
predict_dt = 0.05 #0.1 #0.05 #0.01
optimize_dt = 0.05

# custom optimizer
iter_adam_custom = 1 #300
custom_gd_lr_rate = 0.1
grad_clip = 1.0

# scipy optimizer
iter_scipy=1 #4000

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
def predict_state_ideal(state, policy_params, key, h):
    t = h * predict_dt
    control_input, pos_ref, vel_ref = policy( t, state, policy_params )         # mean_position = get_mean( states, weights )
    next_state, _ = get_next_states_ideal( state, control_input, dt )
    state_ref = jnp.append(pos_ref, vel_ref, axis=0)
    disturbance_mean, disturbance_cov = jnp.zeros((3,1)), jnp.zeros((3,1))
    return next_state, state_ref, control_input, disturbance_mean, disturbance_cov

@jit
def predict_state_noisy(state, policy_params, key, h):

    t = h * predict_dt
    control_input, pos_ref, vel_ref = policy( t, state, policy_params )         # mean_position = get_mean( states, weights )
    next_state, next_state_cov, disturbance_mean, disturbance_cov = get_next_states_noisy_predict( state, control_input, dt )
    key, subkey = jax.random.split(key)
    next_state = next_state + jax.random.normal( subkey, shape=(6,1) ) * jnp.sqrt( next_state_cov )
    # next_states = next_states + jax.random.normal( subkey, shape=(6,13) ) * jnp.sqrt( next_states_cov )
    state_ref = jnp.append(pos_ref, vel_ref, axis=0)
    return next_state, state_ref, control_input, disturbance_mean, disturbance_cov

# @jit
def setup_predict_state_gp(file_path1, file_path2, file_path3, gp_train_x, gp_train_y): #, dt):

    gp0 = initialize_gp_prediction( file_path1 ) #, gp_train_x, gp_train_y[:,0].reshape(-1,1) )
    gp1 = initialize_gp_prediction( file_path2 ) #, gp_train_x, gp_train_y[:,1].reshape(-1,1) )
    gp2 = initialize_gp_prediction( file_path3 ) #, gp_train_x, gp_train_y[:,2].reshape(-1,1) )

    x = gp_train_x
    y = gp_train_y

    ###### precomputes all necessary inverses to save time ######
    D0 = gpx.Dataset(X=x, y=y[0].reshape(-1,1))
    D1 = gpx.Dataset(X=x, y=y[1].reshape(-1,1))
    D2 = gpx.Dataset(X=x, y=y[2].reshape(-1,1))

    L0, L0_inv, Lz0, Lz_inv0, Kzz_inv_Kzx_diff0 = gp0.compute_sigma_inv(train_data=D0)
    L1, L1_inv, Lz1, Lz_inv1, Kzz_inv_Kzx_diff1 = gp1.compute_sigma_inv(train_data=D1)
    L2, L2_inv, Lz2, Lz_inv2, Kzz_inv_Kzx_diff2 = gp2.compute_sigma_inv(train_data=D2)

    @jit
    def predict_state(state, policy_params, key, h):
        
        t = h * predict_dt
        control_input, pos_ref, vel_ref = policy( t, state, policy_params )         # mean_position = get_mean( states, weights )
        next_state, next_state_cov, disturbance_mean, disturbance_cov = get_next_states_with_sparse_gp_sigma_inv_predict( state, control_input, predict_dt, [gp0, gp1, gp2], [L0, L1, L2], [L0_inv, L1_inv, L2_inv], [Lz0, Lz1, Lz2], [Lz_inv0, Lz_inv1, Lz_inv2], [Kzz_inv_Kzx_diff0, Kzz_inv_Kzx_diff1, Kzz_inv_Kzx_diff2])
        key, subkey = jax.random.split(key)
        next_state = next_state + jax.random.normal( subkey, shape=(6,1) ) * jnp.sqrt( next_state_cov )
        state_ref = jnp.append(pos_ref, vel_ref, axis=0)
        return next_state, state_ref, control_input, disturbance_mean, disturbance_cov
    
    return predict_state


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
    L0, L0_inv, Lz0, Lz_inv0, Kzz_inv_Kzx_diff0 = gp0.compute_sigma_inv(train_data=D0)
    L1, L1_inv, Lz1, Lz_inv1, Kzz_inv_Kzx_diff1 = gp1.compute_sigma_inv(train_data=D1)
    L2, L2_inv, Lz2, Lz_inv2, Kzz_inv_Kzx_diff2 = gp2.compute_sigma_inv(train_data=D2)

    @jit
    def compute_reward(X, policy_params, init_time):
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
            t = init_time + h * optimize_dt
            reward, states, weights = inputs
            control_inputs, pos_ref, vel_ref = policy( t, states, policy_params )         # mean_position = get_mean( states, weights )
            if dynamics_type=='ideal':
                next_states_mean, next_states_cov = get_next_states_ideal( states, control_inputs, optimize_dt )
            elif dynamics_type=='noisy':
                next_states_mean, next_states_cov = get_next_states_noisy( states, control_inputs, optimize_dt )
            elif dynamics_type=='gp':
                next_states_mean, next_states_cov = get_next_states_with_sparse_gp_sigma_inv( states, control_inputs, optimize_dt, [gp0, gp1, gp2], [L0, L1, L2], [L0_inv, L1_inv, L2_inv],  [Lz0, Lz1, Lz2], [Lz_inv0, Lz_inv1, Lz_inv2], [Kzz_inv_Kzx_diff0, Kzz_inv_Kzx_diff1, Kzz_inv_Kzx_diff2], [D0, D1, D2])
            next_states_expanded, next_weights_expanded = sigma_point_expand_with_mean_cov( next_states_mean, next_states_cov, weights)
            next_states, next_weights = sigma_point_compress( next_states_expanded, next_weights_expanded )
            states = next_states
            weights = next_weights
            reward = reward + reward_func( states, weights, pos_ref, vel_ref ) # reward is loss
            return reward, states, weights
        reward =  lax.fori_loop( 0, horizon, body, (reward, states, weights) )[0]
        return reward
    return compute_reward

# print(model_path)
# file_path1 = model_path + "sparsegp_model_x_norm5_clipped_moredata.pkl"
# file_path2 = model_path + "sparsegp_model_y_norm5_clipped_moredata.pkl"
# file_path3 = model_path + "sparsegp_model_z_norm5_clipped_moredata.pkl"




# gp_train_x = jnp.load(input_path)
# # gp_train_x = gp_train_x[::80]
# gp_train_x = gp_train_x[::140]
# gp_train_y = jnp.load(disturbance_path)
# gp_train_y = gp_train_y[::140].T

home_path_op = '/home/colcon_ws/src/online_op/online_op/'
gp_file_path = home_path_op+'gp_models/'
trainset_file_path = home_path_op+'dataset/'
disturbance_path = trainset_file_path + 'disturbance_new.npy'
input_path = trainset_file_path + 'input_new.npy'

gp_train_x = jnp.load(input_path)
gp_train_x = gp_train_x[::140]
gp_train_y = jnp.load(disturbance_path)
gp_train_y = gp_train_y[::140].T

file_path1 = gp_file_path + 'sparsegp_model_x_norm5_clipped_moredata.pkl'
file_path2 = gp_file_path + 'sparsegp_model_y_norm5_clipped_moredata.pkl'
file_path3 = gp_file_path + 'sparsegp_model_z_norm5_clipped_moredata.pkl'




# self.gp0 = initialize_gp_prediction(gp_file_x)
# self.gp1 = initialize_gp_prediction(gp_file_y)
# self.gp2 = initialize_gp_prediction(gp_file_z)



# train_x = jnp.load(trainset_file_path + 'training_disturbance_x.npy')
# train_y = jnp.load(trainset_file_path + 'training_disturbance_y.npy')
# train_z = jnp.load(trainset_file_path + 'training_disturbance_z.npy')
# gp_train_x = jnp.load(trainset_file_path+'training_input.npy')
# gp_train_y = jnp.column_stack((train_x, train_y, train_z))

# import pdb
# pdb.set_trace()
t0 = time.time()
    
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
        return predict_state_ideal
    elif dynamics_type=='noisy':
        return predict_state_noisy
    elif dynamics_type=='gp':
        predict_states_gp = setup_predict_state_gp(file_path1, file_path2, file_path3, gp_train_x, gp_train_y)
        return predict_states_gp

# first run
get_future_reward = setup_future_reward_func(file_path1, file_path2, file_path3, dynamics_type=dynamics_type)
predict_state = setup_predict_states(file_path1, file_path2, file_path3, dynamics_type=dynamics_type)
get_future_reward_grad = jit(grad(get_future_reward, argnums=1))
get_future_reward_value_and_grad = jit(value_and_grad(get_future_reward, argnums=1))

get_future_reward( state_vector, jnp.array([7.0, 4.0]), 0.0 )


print(f"grads: {get_future_reward_grad( jnp.zeros((6,1)), jnp.array([7.0, 4.0]), 0.0 )}")
exit()
params_init = jnp.array([7.0, 4.0])
key, subkey = jax.random.split(key)


@jit
def train_policy_jaxscipy(init_state, params_policy):
    minimize_func = lambda params: get_future_reward( init_state, params )
    solver = jaxopt.ScipyMinimize(fun=minimize_func, maxiter=iter_scipy)
    params_policy, cost_state = solver.run(params_policy)
    return params_policy

@jit
def train_policy_custom(init_state, params_policy, init_time):

    @jit
    def body(i, inputs):
        params_policy = inputs
        params_policy_grad = get_future_reward_grad( init_state, params_policy, init_time )
        jax.debug.print("grads {x}", x=params_policy_grad)
        params_policy_grad = jnp.clip( params_policy_grad, -grad_clip, grad_clip )
        params_policy = params_policy - custom_gd_lr_rate * params_policy_grad
        return params_policy
    params_policy = lax.fori_loop(0, iter_adam_custom, body, params_policy)


    # for i in range(iter_adam_custom):
    #     params_policy_grad = get_future_reward_grad( init_state, params_policy, gp_train_x, gp_train_y )
    #     params_policy_grad = jnp.clip( params_policy_grad, -grad_clip, grad_clip )
    #     params_policy = params_policy - custom_gd_lr_rate * params_policy_grad
    return params_policy


def predict_states(init_state, policy_params, key, run_optimizer=False):
    states = init_state
    for h in range(simT):
        key, subkey = jax.random.split(key)
        next_state, state_ref, _, _, _ = predict_state( states[:,[h]], policy_params, subkey, h )
        if h==0:
            states_ref = state_ref
        else:
            states_ref = np.append(states_ref, state_ref, axis=1)
        if run_optimizer:
            if optimizer=='scipy':
                policy_params = train_policy_jaxscipy(next_state, policy_params)
            elif optimizer=='custom_gd':
                policy_params = train_policy_custom(next_state, policy_params,h*predict_dt)
                print(f"params: {policy_params}")
            else:
                print(f"NOT IMPLEMENTED ERROR")
                exit()
        states = np.append(states, next_state, axis=1)
    return states, states_ref


# Unoptimized Parameters
states, states_ref = predict_states(state_vector, jnp.copy(params_init), subkey, run_optimizer=False)
key, subkey = jax.random.split(key)
states2, states_ref2 = predict_states(state_vector, jnp.copy(params_init), subkey, run_optimizer=False)
fig, ax = plt.subplots()
ax.plot(states_ref[0,:], states_ref[1,:], 'r', label='reference')
ax.plot(states[0,:], states[1,:], 'g', label='states unoptimized')
ax.plot(states2[0,:], states2[1,:], 'g--', label='states2 unoptimized')
ax.set_xlabel('X')
ax.set_ylabel('Y')
# plt.show()


print(f"Optimizing now!")
key, subkey = jax.random.split(key)
states_optimized, states_ref_optimized= predict_states(state_vector, jnp.copy(params_init), subkey, run_optimizer=True)
key, subkey = jax.random.split(key)
states_optimized2, states_ref_optimized2 = predict_states(state_vector, jnp.copy(params_init), subkey, run_optimizer=True)
ax.plot(states_optimized[0,:], states_optimized[1,:], 'k', label='states optimized')
ax.plot(states_optimized2[0,:], states_optimized2[1,:], 'k--', label='states2 optimized')



ax.legend()
# ax_acc[0].legend()
# ax_acc[1].legend()
# ax_acc[2].legend()
# plt.savefig(f"gain_tuning/plots/component_wx{w1}_wv{w2}.png")
plt.show()







# fig_acc, ax_acc = plt.subplots(3)
# ax_acc[0].plot( control_inputs[0,:], 'g', label='Control acceleration' )
# ax_acc[1].plot( control_inputs[1,:], 'g' )
# ax_acc[2].plot( control_inputs[2,:], 'g' )
# ax_acc[0].plot( disturbance_means[0,:], 'r', label='Disturbance' )
# ax_acc[1].plot( disturbance_means[1,:], 'r' )
# ax_acc[2].plot( disturbance_means[2,:], 'r' )
# ax_acc[0].plot( control_inputs2[0,:], 'g--')
# ax_acc[1].plot( control_inputs2[1,:], 'g--' )
# ax_acc[2].plot( control_inputs2[2,:], 'g--' )
# ax_acc[0].plot( disturbance_means2[0,:], 'r--')
# ax_acc[1].plot( disturbance_means2[1,:], 'r--' )
# ax_acc[2].plot( disturbance_means2[2,:], 'r--' )
# ax_acc[0].set_ylabel('X')
# ax_acc[1].set_ylabel('Y')
# ax_acc[2].set_ylabel('Z')

# ax_acc[0].plot( control_inputs_optimized[0,:], 'b', label='Control acceleration optimized' )
# ax_acc[1].plot( control_inputs_optimized[1,:], 'b' )
# ax_acc[2].plot( control_inputs_optimized[2,:], 'b' )
# ax_acc[0].plot( disturbance_means_optimized[0,:], 'k', label='Disturbance optimized' )
# ax_acc[1].plot( disturbance_means_optimized[1,:], 'k' )
# ax_acc[2].plot( disturbance_means_optimized[2,:], 'k' )
# ax_acc[0].plot( control_inputs_optimized2[0,:], 'b--', label='Control acceleration optimized' )
# ax_acc[1].plot( control_inputs_optimized2[1,:], 'b--' )
# ax_acc[2].plot( control_inputs_optimized2[2,:], 'b--' )
# ax_acc[0].plot( disturbance_means_optimized2[0,:], 'k--', label='Disturbance optimized' )
# ax_acc[1].plot( disturbance_means_optimized2[1,:], 'k--' )
# ax_acc[2].plot( disturbance_means_optimized2[2,:], 'k--' )