import jax
import jax.numpy as jnp
jax.config.update("jax_enable_x64", True)
import optax as ox
import gpjax as jgp
import pickle
from gpjax import Dataset
from jax import jit, random
key = random.PRNGKey(2)
home_path = '/Users/albusfang/Coding Projects/gp_ws/Gaussian Process/GP/gp_advanced/FORESEE/'

@jit
def get_next_states_ideal(states, control_inputs, dt):
    # double integertor dynamics

    m = 0.681 #kg
    g = 9.8066

    states_next_pos = states[0:3] + states[3:6] * dt

    states_next_vel = states[3:6] + control_inputs * dt + g * dt * jnp.array([ [0], [0], [1] ])

    states_next = jnp.append( states_next_pos, states_next_vel, axis=0 )
    cov_next = jnp.zeros( (6,13) )

    return states_next, cov_next

factor = 3.0 #0.3 #0.01 #0.01

@jit
def get_next_states_noisy_predict(states, control_inputs, dt):
    # double integertor dynamics

    m = 0.681 #kg
    g = 9.8066

    states_next_pos = states[0:3] + states[3:6] * dt
    
    drag_mean = - factor * states[3:6]/jnp.maximum(0.01*jnp.ones(1),jnp.linalg.norm(states[3:6], axis=0)) * jnp.square(states[3:6])
    drag_cov = 0.005 * jnp.ones( (3,1) )

    states_next_vel = states[3:6] + control_inputs * dt + g * dt * jnp.array([ [0], [0], [1] ]) + drag_mean * dt

    states_next = jnp.append( states_next_pos, states_next_vel, axis=0 )
    cov_next = jnp.append( jnp.zeros( (3,1) ), drag_cov, axis=0)

    return states_next, cov_next, drag_mean, drag_cov

@jit
def get_next_states_noisy(states, control_inputs, dt):
    # double integertor dynamics

    m = 0.681 #kg
    g = 9.8066

    states_next_pos = states[0:3] + states[3:6] * dt

    drag_mean = - factor * states[3:6]/jnp.maximum(0.01*jnp.ones(13),jnp.linalg.norm(states[3:6], axis=0)) * jnp.square(states[3:6])
    drag_cov = 0.005 * jnp.ones( (3,13) )

    states_next_vel = states[3:6] + control_inputs * dt + g * dt * jnp.array([ [0], [0], [1] ]) + drag_mean * dt

    states_next = jnp.append( states_next_pos, states_next_vel, axis=0 )
    cov_next = jnp.append( jnp.zeros( (3,13) ), drag_cov, axis=0)

    return states_next, cov_next

@jit
def get_next_states_with_gp_sigma_inv( states, control_inputs, dt, gps, sigma_inv, train_x, train_y ):
    
    '''
    Propogate sigma points through the nonliear GP
    '''
    test_x = states.T #jnp.append( states.T, control_inputs.T, axis=0)
    g = 9.8066

    #################################################
    ####### Changed: dataset(.reshape) #######
    # X disturbance
    #D = Dataset(X=train_x, y=train_y[0])
    D = Dataset(X=train_x, y=train_y[0].reshape(-1,1))
    latent_dist = gps[0].predict_with_sigma_inv(test_x, D, Sigma_inv=sigma_inv[0])
    predictive_dist = gps[0].likelihood(latent_dist)
    pred_mean0 = predictive_dist.mean().reshape(-1,1)
    pred_std0 = predictive_dist.stddev().reshape(-1,1)

    # Y disturbance
    #D = Dataset(X=train_x, y=train_y[1])
    D = Dataset(X=train_x, y=train_y[1].reshape(-1,1))
    latent_dist = gps[1].predict_with_sigma_inv(test_x, D, sigma_inv[1])
    predictive_dist = gps[1].likelihood(latent_dist)
    pred_mean1 = predictive_dist.mean().reshape(-1,1)
    pred_std1 = predictive_dist.stddev().reshape(-1,1)

    # Z disturbance
    #D = Dataset(X=train_x, y=train_y[2])
    D = Dataset(X=train_x, y=train_y[2].reshape(-1,1))
    latent_dist = gps[2].predict_with_sigma_inv(test_x, D, sigma_inv[2])
    predictive_dist = gps[2].likelihood(latent_dist)
    pred_mean2 = predictive_dist.mean().reshape(-1,1)
    pred_std2 = predictive_dist.stddev().reshape(-1,1)

    pred_mu = jnp.concatenate( (pred_mean0.T, pred_mean1.T, pred_mean2.T), axis=0 ) #3x13
    pred_cov = jnp.concatenate( (pred_std0.T**2, pred_std1.T**2, pred_std2.T**2), axis=0 ) #3x13
    ################################################
    ############ bug fix: ########################
    ############ bug: line 52, incompatible shapes control inputs and pred_mu ############
    #pred_mu = pred_mu.reshape(3,-1)
    
    ################################################
    next_states_pos = states[0:3] + states[3:6] * dt #+ control_inputs * dt**2/2
    next_states_vel_mu = states[3:6] + control_inputs * dt + g * dt + pred_mu * dt
    next_states_vel_cov = pred_cov * dt * dt

    next_states_mu = jnp.append( next_states_pos, next_states_vel_mu, axis=0 )
    next_states_cov = jnp.append( jnp.zeros((3,13)), next_states_vel_cov, axis=0 )
    return next_states_mu, next_states_cov

@jit
def get_next_states_with_gp_sigma_inv_predict( states, control_inputs, dt, gps, sigma_inv, train_x, train_y ):
    
    '''
    Propogate sigma points through the nonliear GP
    '''
    test_x = states.T #jnp.append( states.T, control_inputs.T, axis=0)
    g = 9.8066

    #################################################
    ####### Changed: dataset(.reshape) #######
    # X disturbance
    #D = Dataset(X=train_x, y=train_y[0])
    D = Dataset(X=train_x, y=train_y[0].reshape(-1,1))
    latent_dist = gps[0].predict_with_sigma_inv(test_x, D, Sigma_inv=sigma_inv[0])
    predictive_dist = gps[0].likelihood(latent_dist)
    pred_mean0 = predictive_dist.mean().reshape(-1,1)
    pred_std0 = predictive_dist.stddev().reshape(-1,1)

    # Y disturbance
    #D = Dataset(X=train_x, y=train_y[1])
    D = Dataset(X=train_x, y=train_y[1].reshape(-1,1))
    latent_dist = gps[1].predict_with_sigma_inv(test_x, D, sigma_inv[1])
    predictive_dist = gps[1].likelihood(latent_dist)
    pred_mean1 = predictive_dist.mean().reshape(-1,1)
    pred_std1 = predictive_dist.stddev().reshape(-1,1)

    # Z disturbance
    #D = Dataset(X=train_x, y=train_y[2])
    D = Dataset(X=train_x, y=train_y[2].reshape(-1,1))
    latent_dist = gps[2].predict_with_sigma_inv(test_x, D, sigma_inv[2])
    predictive_dist = gps[2].likelihood(latent_dist)
    pred_mean2 = predictive_dist.mean().reshape(-1,1)
    pred_std2 = predictive_dist.stddev().reshape(-1,1)

    pred_mu = jnp.concatenate( (pred_mean0.T, pred_mean1.T, pred_mean2.T), axis=0 ) #3x13
    pred_cov = jnp.concatenate( (pred_std0.T**2, pred_std1.T**2, pred_std2.T**2), axis=0 ) #3x13
    ################################################
    ############ bug fix: ########################
    ############ bug: line 52, incompatible shapes control inputs and pred_mu ############
    #pred_mu = pred_mu.reshape(3,-1)
    
    ################################################
    next_states_pos = states[0:3] + states[3:6] * dt #+ control_inputs * dt**2/2
    next_states_vel_mu = states[3:6] + control_inputs * dt + g * dt * jnp.array([ [0], [0], [1] ]) + pred_mu * dt
    next_states_vel_cov = pred_cov * dt * dt

    next_states_mu = jnp.append( next_states_pos, next_states_vel_mu, axis=0 )
    next_states_cov = jnp.append( jnp.zeros((3,1)), next_states_vel_cov, axis=0 )
    return next_states_mu, next_states_cov, pred_mu, pred_cov

def train_gp(likelihood, posterior, parameter_state, train_x, train_y):
    D = Dataset(X=train_x, y=train_y)
    negative_mll = jit(posterior.marginal_log_likelihood(D, negative=True))
    optimiser = ox.adam(learning_rate=0.08)
    inference_state = jgp.fit(
        objective=negative_mll,
        parameter_state=parameter_state,
        optax_optim=optimiser,
        num_iters=1500,
    )
    learned_params, training_history = inference_state.unpack()
    return likelihood, posterior, learned_params, D

def initialize_gp_prediction(file_path):#, train_x, train_y): #, test_x):
    def load_gp_model(file_path):
        with open(file_path, 'rb') as f:
            opt_posterior = pickle.load(f)
        return opt_posterior

    # latent_dist = posterior(gp_params, D)
    # D = Dataset(X=train_x, y=train_y)
    opt_posterior = load_gp_model(file_path)
    return opt_posterior