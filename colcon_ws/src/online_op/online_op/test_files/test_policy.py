import jax.numpy as jnp
from jax import jit, vmap, lax
import jax
def state_ref(t):
    dataset_index = 1
    pos, vel, acc = figure8_pos_vel_acc( t, figure8_radius[dataset_index], figure8_angular_vel[dataset_index], figure8_origin_x[dataset_index], figure8_origin_y[dataset_index] )
    return pos.reshape(-1,1), vel.reshape(-1,1), acc.reshape(-1,1)
# policy_params = [14, 7.4]
policy_params = [7, 4]
@jit
def policy( t, states, policy_params):
    '''
    Expect a multiple states as input. Each state is a column vector.
    Should then return multiple control inputs. Each input should be a column vector
    '''
    m = 0.681 #kg
    g = 9.8066
    # kx = 14
    # kv = 7.4

    kx = policy_params[0]
    kv = policy_params[1]

    pos_ref, vel_ref, acc_ref = state_ref(t)

    ex = states[0:3] - pos_ref
    # ex = lax.cond( jnp.linalg.norm(ex)>2, lambda z: 2.0 * z / jnp.linalg.norm(z), lambda z: z, ex )
    ev = states[3:6] - vel_ref
    # ev = lax.cond( jnp.linalg.norm(ev)>5, lambda z: 5.0 * z / jnp.linalg.norm(z), lambda z: z, ev )
    # thrust = - kx * ex - kv * ev + m * acc_ref - m * g
    thrust = - kx * ex - kv * ev + m * acc_ref - m * g * jnp.array([ [0], [0], [1] ])
    tanh_a = 3.8
    tanh_k = 0.286*1.5


    tanh_az = 15*0.681
    tanh_kz = 0.1
    
    # jax.debug.print("acc = {acc}",acc=thrust/m)
    # jax.debug.print("thrust shape = {shape}", shape = thrust.shape)
    thrust = thrust.at[0,0].set( tanh_a * jnp.tanh( tanh_k * thrust[0,0] ) )
    thrust = thrust.at[1,0].set( tanh_a * jnp.tanh( tanh_k * thrust[1,0] ) )
    thrust = thrust.at[2,0].set( tanh_az * jnp.tanh( tanh_kz * thrust[2,0] ) )
    # thrust = jnp.clip( thrust, -14*m, 14*m ) #tanh/sigmoid
    return thrust / m, pos_ref, vel_ref

cir_radius =      [0.2, 0.2, 0.2, 0.2, 0.4]
cir_angular_vel = [1.0, 1.0, 1.0, 1.5, 1.0]
###### in world frame NOT NED ######
cir_origin_x =    [0.0, 0.6, 0.8, 0.8, 0.6]
cir_origin_y =    [0.0, 0.0, 0.0, 0.0, 0.0]

# cir_radius =      [0.2, 0.3, 0.4, 0.4, 0.4, 0.4]
# cir_angular_vel = [1.5, 1.5, 2.0, 2.5, 3.0, 3.0]
# ###### in world frame NOT NED ######
# cir_origin_x =    [0.5, 0.4, 0.4, 0.6, 0.8, 1.0]
# cir_origin_y =    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

figure8_radius =        [0.4, 0.4]
figure8_angular_vel =   [1, 1]
figure8_origin_x =      [0.0, 0.8]
figure8_origin_y =      [0.0, 0.0]

# figure8_radius =        [0.2, 0.2, 0.2, 0.4, 0.4, 0.4, 0.4, 0.4, 0.6]
# figure8_angular_vel =   [1.5, 2.0, 2.5, 1.5, 1.5, 2.0, 2.0, 2.5, 1.5]
# figure8_origin_x =      [0.8, 1.2, 1.2, 1.2, 1.0, 0.4, 1.0, 1.0, 1.0]
# figure8_origin_y =      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


@jit
def circle_pos_vel_acc(deltaT, radius, angular_vel, origin_x, origin_y):
    '''
    Calculate reference pos, vel, and acc for the drone flying in a circular trajectory in NED frame.
    '''

    ######################################################
    ################## Reference Pos #####################
    ######################################################
    x = radius * jnp.cos(angular_vel * deltaT) + origin_x
    y = radius * jnp.sin(angular_vel * deltaT) + origin_y
    ref_pos = jnp.array([y,  x, -0.4])

    ######################################################
    ################## Reference Vel #####################
    ######################################################
    vx = -radius * angular_vel* jnp.sin(angular_vel * deltaT)
    vy = radius * angular_vel* jnp.cos(angular_vel * deltaT)
    ref_vel = jnp.array([vy, vx,0])

    ######################################################
    ################## Reference Acc #####################
    ######################################################
    ax = -radius * (angular_vel**2) * jnp.cos(angular_vel * deltaT)
    ay = -radius * (angular_vel**2) * jnp.sin(angular_vel * deltaT)
    ref_acc = jnp.array([ay, ax, 0])

    ######################################################
    ################## To Jnp Array ######################
    ######################################################
    # assert pos.shape == vel.shape == acc.shape, "output shapes are different"
    return ref_pos, ref_vel, ref_acc



def figure8_pos_vel_acc(deltaT, radius, angular_vel, origin_x, origin_y):
    '''
    Calculate reference pos, vel, and acc for the drone flying in figure8 trajectory in NED frame
    '''
    ######################################################
    ################## Reference Pos #####################
    ######################################################
    x = radius * jnp.sin(angular_vel * deltaT) + origin_x
    y = radius * jnp.sin(angular_vel * deltaT) * jnp.cos(angular_vel * deltaT) + origin_y
    ref_pos = [y,  x, -0.4]

    ######################################################
    ################## Reference Vel #####################
    ######################################################
    vx = radius * angular_vel * jnp.cos(angular_vel * deltaT)
    vy = radius * angular_vel * (jnp.cos(angular_vel * deltaT)**2-jnp.sin(angular_vel * deltaT)**2)
    ref_vel= [vy,vx,0]
    ######################################################
    ################## Reference Acc #####################
    ######################################################
    ax = -radius * (angular_vel**2) * jnp.sin(angular_vel * deltaT)
    ay = -radius * 4 * (angular_vel**2) * jnp.sin(angular_vel * deltaT) * jnp.cos(angular_vel * deltaT)
    ref_acc = [ay,ax,0]
    pos = jnp.array(ref_pos)
    vel = jnp.array(ref_vel)
    acc = jnp.array(ref_acc)
    assert pos.shape == vel.shape == acc.shape, "output shapes are different"
    return pos, vel, acc



def generate_time(start_time, end_time, step_size):
    '''
    Generate a time vector to simulate trajectories
    '''
    return jnp.arange(start_time, end_time + step_size, step_size)



def generate_reference_circle(time_array, cir_radius, cir_angular_vel, cir_origin_x, cir_origin_y):
    '''
    Generate circle trajectories of different parameters for a specified time period
    '''
    ref_pos = ref_vel = ref_acc = jnp.empty((0,3))
    
    vectorized_circle_pos_vel_acc = vmap(circle_pos_vel_acc, in_axes=(0, None, None, None, None))
    for radius, angular_vel, origin_x, origin_y in zip(cir_radius,cir_angular_vel,cir_origin_x,cir_origin_y):
        pos_arr, vel_arr, acc_arr = vectorized_circle_pos_vel_acc(time_array, radius, angular_vel, origin_x, origin_y)
        assert pos_arr.shape[1] == vel_arr.shape[1] == acc_arr.shape[1] == 3, "missing dim"
        assert pos_arr.shape[0] == vel_arr.shape[0] == acc_arr.shape[0] == time_array.shape[0], "first dim"
       
        ref_pos = jnp.vstack((ref_pos, pos_arr))
        ref_vel = jnp.vstack((ref_vel, vel_arr))
        ref_acc = jnp.vstack((ref_acc, acc_arr))
        

    return ref_pos, ref_vel, ref_acc
def generate_reference_vectorize(trajectory, time_array, cir_radius, cir_angular_vel, cir_origin_x, cir_origin_y):
    '''
    Generate reference pos, vel, and acc in a given time period.

    trajectory: either circle_pos_vel_acc or figure8_pos_vel_acc
    
    '''
    def single_trajectory_params(trajectory, radius, angular_vel, origin_x, origin_y):
        vectorized_circle_pos_vel_acc = vmap(trajectory, in_axes=(0, None, None, None, None))
        return vectorized_circle_pos_vel_acc(time_array, radius, angular_vel, origin_x, origin_y)
    
    vectorized_trajectories = vmap(single_trajectory_params, in_axes=(None, 0, 0, 0, 0))
    all_pos, all_vel, all_acc = vectorized_trajectories(trajectory, jnp.array(cir_radius), jnp.array(cir_angular_vel), jnp.array(cir_origin_x), jnp.array(cir_origin_y))

    ref_pos = all_pos.reshape(-1, 3)
    ref_vel = all_vel.reshape(-1, 3)
    ref_acc = all_acc.reshape(-1, 3)
    
    return ref_pos, ref_vel, ref_acc
# def generate_reference_circle_vectorize(time_array, cir_radius, cir_angular_vel, cir_origin_x, cir_origin_y):
#     def single_trajectory_params(radius, angular_vel, origin_x, origin_y):
#         vectorized_circle_pos_vel_acc = vmap(circle_pos_vel_acc, in_axes=(0, None, None, None, None))
#         return vectorized_circle_pos_vel_acc(time_array, radius, angular_vel, origin_x, origin_y)
    
#     vectorized_trajectories = vmap(single_trajectory_params, in_axes=(0, 0, 0, 0))
#     all_pos, all_vel, all_acc = vectorized_trajectories(jnp.array(cir_radius), jnp.array(cir_angular_vel), jnp.array(cir_origin_x), jnp.array(cir_origin_y))

#     ref_pos = all_pos.reshape(-1, 3)
#     ref_vel = all_vel.reshape(-1, 3)
#     ref_acc = all_acc.reshape(-1, 3)
    
#     return ref_pos, ref_vel, ref_acc

time_array = generate_time(0, 5, 0.1)

ref_pos, ref_vel, ref_acc = generate_reference_circle(time_array,cir_radius, cir_angular_vel, cir_origin_x, cir_origin_y)
vec_pos, vec_vel, vec_acc = generate_reference_vectorize(circle_pos_vel_acc, time_array, cir_radius, cir_angular_vel, cir_origin_x, cir_origin_y)
print("Positions match:", jnp.allclose(ref_pos, vec_pos))
print("Velocities match:", jnp.allclose(ref_pos, vec_pos))
print("Accelerations match:", jnp.allclose(ref_acc, vec_acc))


# assert jnp.array_equal(ref_pos, vec_pos) and jnp.array_equal(ref_vel,vec_vel) and jnp.array_equal(ref_acc, vec_acc)
