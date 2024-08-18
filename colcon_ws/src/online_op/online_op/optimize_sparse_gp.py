#!/usr/bin/env python3
import rclpy
import rclpy.node
import sys, os
from std_msgs.msg import Bool
from functools import partial
current_dir = os.path.dirname(os.path.abspath(__file__))
# current_dir = os.getcwd()
# sys.path.append('./GPJax')
home_path_op = '/home/colcon_ws/src/online_op/online_op/'
#sys.path.append(current_dir + '/GPJax/')
sys.path.append(home_path_op+'GPJax')
# print(home_path)
# print(sys.path)

#print(current_dir+'/GPjax/')
import gpjax as gpx
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, ParameterReq
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from .jax_utils import *
from .gp_utils_sparse import *
from .policy import *
from foresee_msgs.msg import TrajectoryInfo
from pymavlink import mavutil
from .optimize_helper import *
from jax import grad, jit, lax, value_and_grad, jacrev, jacfwd
from jax.experimental import host_callback as hcb
def print_debug(value, msg):
    def _print(x):
        print(f"{msg}: {x}")
    return hcb.call(_print, value, result_shape=value)

# os.environ['JAX_TRACEBACK_FILTERING'] = 'off'
# import pymavparam as pm

class optimizer(Node):

    get_future_reward = None
    get_future_reward_grad = None
    grad_clip = 20
    custom_lr_rate = 0.1
    iter_adam_custom = 1
    def __init__(self):
        super().__init__('optimizer')

        qos_profile = QoSProfile(
                            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                            depth=1
        )

        
        ###### set up trajectory parameters ######
        self.get_logger().info('Optimizer Node Starts')
        


        # self.declare_parameter('trajectory_type', 'circle')
        # self.trajectory_type = self.get_parameter('trajectory_type').get_parameter_value().string_value
        # self.get_logger().info(f'Trajectory type: {self.trajectory_type}')
        # self.trajectory_type = 'circle'
        # self.radius = 0.4
        # self.height = -0.5
        # self.center_x = 0.6
        # self.center_y = 0.0
        # self.angular_vel = 1.0
        self.trajectory_type = None
        self.trajectory_type_int = 0
        self.radius = 0.4 #None
        self.height = -0.55 #None
        self.center_x = 0.0 #None
        self.center_y = 0.0 #None
        self.angular_vel = 1.0 #None
        self.trajectory_type_valid = False
        self.horizon = 50

        ###### set up initial drone parameters ######
        self.kx = 7.0
        self.kv = 4.0

        self.current_pos = None
        self.current_vel = None
        self.current_state = None

        self.pos_ref = None
        self.vel_ref = None
        
        self.ref_valid = False


        ###### set up node parameters ######
        
        
        self.clock  = self.get_clock()
        # self.start_time = self.get_clock().now().nanoseconds
        self.start_time = None

        ###### set up Gaussian Process parameters ######
        self.gp0 = None
        self.gp1 = None
        self.gp2 = None
        self.training_state = None
        self.training_disturbance = None
        self.initialize_gp()
                
        ###### set up optimizer parameters ######
        # w1 = 0.5
        # w2 = 0.1
        
        # self.op_dt = 0.05
        optimizer.custom_lr_rate = 0.1
        optimizer.grad_clip = 20.0
        optimizer.iter_adam_custom = 1 #200
        self.optimizer_init = False
        
        ###### set up mavlink ######
        # self.mavlink_ = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        # self.mavlink_ = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
        # self.mavlink_.wait_heartbeat()
        # self.get_logger().info("Mavlink Connected")
        ###### set up publisher for pxhawk ######
        self.publisher_ = self.create_publisher(ParameterReq,'/px4_1/fmu/in/parameter_req',10)

        ################## set up Subscription ##################
        self.drone_coordinates = self.create_subscription(
		    VehicleLocalPosition,
		    '/px4_1/fmu/out/vehicle_local_position',
		    self.coordinate_callback,
            qos_profile=qos_profile)
		    # 10,#)
            
        
        self.trajectory_info = self.create_subscription(
		    TrajectoryInfo,
		    '/drone/TrajectoryInfo',
		    self.trajectory_info_callback,
		    10)
        
        self.optimizer_init_sub = self.create_subscription( Bool, '/optimizer_init', self.optimizer_init_callback, 10 )

        self.timer_period = 0.05
        self.timer_kx = self.create_timer(0.05, self.kx_callback)
        self.timer_ky = self.create_timer(1.0, self.kv_callback)
        self.timer_optimize = self.create_timer(0.05, self.optimize_callback)

        # # Initialize reward function and its gradient
        # optimizer.get_future_reward = self.setup_reward_func()
        # optimizer.get_future_reward_grad = jit(grad(optimizer   .get_future_reward, argnums=(1)))

        # # Run once to JIT
        # params_policy = jnp.array([self.kx, self.kv])
        # init_state = jnp.array([0.0,0,0,0,0,0]).reshape(-1,1)            
        # optimizer.get_future_reward(init_state, params_policy, jnp.array([0]) )
        # optimizer.get_future_reward_grad( init_state, params_policy, jnp.array([0]) )
        # self.get_logger().info("Gaussian Process Initialized")

    def optimizer_init_callback(self, msg):
        self.optimizer_init = msg.data
        
    def trajectory_info_callback(self,msg):
        if self.trajectory_type_valid is False:
            self.trajectory_type = msg.type
            self.trajectory_type_int = 0 if self.trajectory_type == 'circle' else 1
            # print(self.trajectory_type, self.trajectory_type == 'circle')
            self.radius = msg.radius
            self.angular_vel = msg.angular_vel
            self.center_x = msg.center_x
            self.center_y = msg.center_y
            self.height = msg.height
            self.start_time = msg.start_time

            # Initialize reward function and its gradient
            optimizer.get_future_reward = self.setup_reward_func()
            optimizer.get_future_reward_grad = jit(grad(optimizer   .get_future_reward, argnums=(1)))

            # Run once to JIT
            params_policy = jnp.array([self.kx, self.kv])
            init_state = jnp.array([0.0,0,0,0,0,0]).reshape(-1,1)            
            optimizer.get_future_reward(init_state, params_policy, jnp.array([0]) )
            optimizer.get_future_reward_grad( init_state, params_policy, jnp.array([0]) )
            self.get_logger().info("Gaussian Process Initialized")



            self.trajectory_type_valid = True

    def coordinate_callback(self, msg):
            # if self.ref_valid is False:
            #     self.ref_valid = True
            #     self.initialize_gp()
            self.current_pos = [msg.x, msg.y, msg.z]
            self.current_vel = [msg.vx,msg.vy,msg.vz]
            self.current_state = jnp.array(self.current_pos + self.current_vel)
            # print(self.current_state)
            # self.get_logger().info(f'state is:  {self.current_state}')

            # if not self.optimizer_init:
            #     return
            
            # if self.trajectory_type_valid is True:
            #     deltaT = jnp.array([(self.get_clock().now().nanoseconds-self.start_time)/10**9])
            #     # ref_coord = self.find_ref_coord(deltaT)
            #     self.kx, self.kv = self.optimize(deltaT)
            #     # self.get_logger().info(f'QUAD_KX is:  {self.kx} and QUAD_KV is: {self.kv}')
            #     # self.publish_gains()

    def initialize_gp(self):
        self.get_logger().info('Initializing Gaussian Process Models')
        ###### load gaussian process models ######
        gp_file_path = home_path_op+'gp_models/'
        gp_file_x = gp_file_path + 'sparsegp_model_x_norm5_clipped_moredata.pkl'
        gp_file_y = gp_file_path + 'sparsegp_model_y_norm5_clipped_moredata.pkl'
        gp_file_z = gp_file_path + 'sparsegp_model_z_norm5_clipped_moredata.pkl'
        
        ###### load Datasets ######
        trainset_file_path = home_path_op+'dataset/'
        train_x = jnp.load(trainset_file_path + 'training_disturbance_x.npy')
        train_y = jnp.load(trainset_file_path + 'training_disturbance_y.npy')
        train_z = jnp.load(trainset_file_path + 'training_disturbance_z.npy')
        x = jnp.load(trainset_file_path+'training_input.npy')
        y = jnp.column_stack((train_x, train_y, train_z))

        self.gp0 = initialize_gp_prediction(gp_file_x)
        self.gp1 = initialize_gp_prediction(gp_file_y)
        self.gp2 = initialize_gp_prediction(gp_file_z)
        
        trainset_slice = 50
        x = x[::trainset_slice]
        y = y[::trainset_slice].T
        self.training_state = x
        self.training_disturbance = y
        
        D0 = gpx.Dataset(X=x, y=y[0].reshape(-1,1))
        D1 = gpx.Dataset(X=x, y=y[1].reshape(-1,1))
        D2 = gpx.Dataset(X=x, y=y[2].reshape(-1,1))
        ###### compute the inverses ######
        # self.sigma0 = self.gp0.posterior.compute_sigma_inv(train_data=D0)
        # self.sigma1 = self.gp1.posterior.compute_sigma_inv(train_data=D1)
        # self.sigma2 = self.gp2.posterior.compute_sigma_inv(train_data=D2)

        self.L0, self.L0_inv, self.Lz0, self.Lz_inv0, self.Kzz_inv_Kzx_diff0 = self.gp0.compute_sigma_inv(train_data=D0)
        self.L1, self.L1_inv, self.Lz1, self.Lz_inv1, self.Kzz_inv_Kzx_diff1 = self.gp1.compute_sigma_inv(train_data=D1)
        self.L2, self.L2_inv, self.Lz2, self.Lz_inv2, self.Kzz_inv_Kzx_diff2 = self.gp2.compute_sigma_inv(train_data=D2)


    def setup_reward_func(self):

        trajectory_type_int = self.trajectory_type_int
        radius = self.radius
        angular_vel = self.angular_vel
        center_x = self.center_x
        center_y = self.center_y
        height = self.height

        horizon = self.horizon
        gp0, gp1, gp2 = self.gp0, self.gp1, self.gp2

        L0, L0_inv, Lz0, Lz_inv0, Kzz_inv_Kzx_diff0 = self.L0, self.L0_inv, self.Lz0, self.Lz_inv0, self.Kzz_inv_Kzx_diff0
        L1, L1_inv, Lz1, Lz_inv1, Kzz_inv_Kzx_diff1 = self.L1, self.L1_inv, self.Lz1, self.Lz_inv1, self.Kzz_inv_Kzx_diff1
        L2, L2_inv, Lz2, Lz_inv2, Kzz_inv_Kzx_diff2 = self.L2, self.L2_inv, self.Lz2, self.Lz_inv2, self.Kzz_inv_Kzx_diff2


        @jit
        def compute_reward(X, policy_params, deltaT):
            '''
            Performs Gradient Descent
            '''
            states, weights = initialize_sigma_points(X)
            kx = policy_params[0]
            kv = policy_params[1]
            w1 = 0.5
            w2 = 0.1
            opt_dt = 0.05
            # reward = 0 + w1 * (kx-7)**2 + w2 * (kv-4)**2
            reward = w1 * (kx)**2 + w2 * (kv)**2
            # reward = 0
            def body(h, inputs):
                '''
                Performs UT-EC with 6 states
                '''
                t = deltaT + h * opt_dt
                reward, states, weights = inputs
                ref_pos,ref_vel,ref_acc = find_ref_pos_vel_acc(trajectory_type_int,t,[radius, angular_vel, center_x, center_y, height])
                control_inputs, pos_ref, vel_ref = policy( states, policy_params, [ref_pos,ref_vel,ref_acc])         # mean_position = get_mean( states, weights )
                next_states_mean, next_states_cov = get_next_states_with_sparse_gp_sigma_inv( states, control_inputs, opt_dt, [gp0, gp1, gp2], [L0, L1, L2], [L0_inv, L1_inv, L2_inv],  [Lz0, Lz1, Lz2], [Lz_inv0, Lz_inv1, Lz_inv2], [Kzz_inv_Kzx_diff0, Kzz_inv_Kzx_diff1, Kzz_inv_Kzx_diff2])
                next_states_expanded, next_weights_expanded = sigma_point_expand_with_mean_cov( next_states_mean, next_states_cov, weights)
                next_states, next_weights = sigma_point_compress( next_states_expanded, next_weights_expanded )
                states = next_states
                weights = next_weights
                reward = reward + reward_func( states, weights, pos_ref, vel_ref ) # reward is loss
                return reward, states, weights
            reward =  lax.fori_loop( 0, horizon, body, (reward, states, weights) )[0]
            return reward
        return compute_reward

    
    def optimize_scipy(self, deltaT):
        print("Optimizing with Scipy")
        gp_train_x = self.training_state
        
        gp_train_y = self.training_disturbance
        params_policy = jnp.array([self.kx, self.kv])
        init_state = jnp.array(self.current_state)
        ref_pos,ref_vel,ref_acc = self.find_ref_pos_vel_acc(deltaT)


    def optimize_callback(self):
        if not self.optimizer_init:
            return
        if self.trajectory_type_valid is True:
            self.get_logger().info(f'optimizing')
            deltaT = jnp.array([(self.get_clock().now().nanoseconds-self.start_time)/10**9])

            params_policy = jnp.array([self.kx, self.kv])
            init_state = jnp.array(self.current_state)

            t0 = self.get_clock().now().nanoseconds
            kx, kv = optimizer.optimize(init_state, deltaT, params_policy)
            self.kx, self.kv = np.clip(kx, 0.01, 30), np.clip(kv, 0.01, 30)
            t1 = self.get_clock().now().nanoseconds
            self.get_logger().info(f"time taken : {(t1-t0)/10**9}")

    @staticmethod
    @jit
    def optimize(init_state, deltaT, params_policy):
        
            # gp_train_x = self.training_state        
            # gp_train_y = self.training_disturbance
            # params_policy = jnp.array([self.kx, self.kv])
            # init_state = jnp.array(self.current_state)
            # print("init state: ", init_state)
            # self.get_logger().info(f"The State Vector is: {init_state}")
            # print("initial state type is: ",type(init_state))
            # print("initial state shape is ", init_state.shape)
            # print("policy params type is: ",type(params_policy))
            # print("gp train type is: ",type(gp_train_x), type(gp_train_y))
            # print("deltaT type is: ",type(deltaT))
            # ref_pos,ref_vel,ref_acc = self.find_ref_pos_vel_acc(deltaT)

            @jit
            def body(i, inputs):
                params_policy = inputs
                params_policy_grad = optimizer.get_future_reward_grad( init_state, params_policy, deltaT )
                params_policy_grad = jnp.clip( params_policy_grad, -optimizer.grad_clip, optimizer.grad_clip )
                params_policy = params_policy - optimizer.custom_lr_rate * params_policy_grad
                return params_policy        
            params_policy = lax.fori_loop(0, optimizer.iter_adam_custom, body, params_policy)
            op_kx = params_policy[0]
            op_kv = params_policy[1]
            return op_kx, op_kv
    
    def find_ref_coord(self, deltaT):
        # if self.trajectory_type == 'circle':
        #     pos_vel_acc = circle_pos_vel_acc
        # else:
        #     pos_vel_acc = figure8_pos_vel_acc
        # ref_coord,_,_ = pos_vel_acc(deltaT, self.radius, self. angular_vel, self.center_x, self.center_y)
        ref_coord,_,_ = self.find_ref_pos_vel_acc(deltaT)
        return ref_coord.reshape(-1,1)
    
    def find_ref_pos_vel_acc(self, deltaT):
        if self.trajectory_type == 'circle':
            pos_vel_acc = circle_pos_vel_acc
        else:
            pos_vel_acc = figure8_pos_vel_acc
        ref_pos,ref_vel,ref_acc = pos_vel_acc(deltaT, self.radius, self.angular_vel, self.center_x, self.center_y)
        return ref_pos,ref_vel,ref_acc
    
    def create_ParameterReq_msg(self, param_name_,value_):
        msg = ParameterReq()
        param_name_char_array = ['']*16
        # print("lenght of param_name_ is: ",len(param_name_))
        for i in range(len(param_name_char_array)):
            if i < len(param_name_):
                param_name_char_array[i] = ord(param_name_[i])
            else:
                param_name_char_array[i] = ord('\0')
        # print(type(param_name_char_array[0]))
        # print("length of param_name_char_array is: ",len(param_name_char_array))
        # param_name_ = param_name_.ljust(16, '\0')
        msg.param_name = param_name_char_array
        msg.set = True
        # print("value_ is: ", value_)
        # print("Type of value is: ", type(value_.item()))
        msg.value = float(value_)
        return msg
    # def publish_gains(self):
        
    #     message_kv = self.create_ParameterReq_msg('QUAD_KV', self.kv)
    #     self.publisher_.publish(message_kv)
    #     self.publisher_.publish(message_kv)

    #     message_kx = self.create_ParameterReq_msg('QUAD_KX', self.kx)
    #     self.publisher_.publish(message_kx)
    #     self.publisher_.publish(message_kx)
        
    def kx_callback(self):
        message_kx = self.create_ParameterReq_msg('QUAD_KX', self.kx)
        self.publisher_.publish(message_kx)
        self.get_logger().info(f'X: QUAD_KX is:  {self.kx} and QUAD_KV is: {self.kv}')

    def kv_callback(self):
        message_kv = self.create_ParameterReq_msg('QUAD_KV', self.kv)
        self.publisher_.publish(message_kv)
        self.get_logger().info(f'Y: QUAD_KX is:  {self.kx} and QUAD_KV is: {self.kv}')
        
    # def publish_optimal_gains(self):
    #     print(self.kx, self.kv)
    #     self.get_logger().info(f'Sending Gains: QUAD_KX = {self.kx}, QUAD_KV = {self.kv}')
    #     self.mavlink_.mav.param_set_send(
    #         self.mavlink_.target_system, self.mavlink_.target_component,
    #         b'QUAD_KX',
    #         self.kx,
    #         mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    #     )

    #     self.mavlink_.mav.param_set_send(
    #         self.mavlink_.target_system, self.mavlink_.target_component,
    #         b'QUAD_KV',
    #         self.kv,
    #         mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    #     )

def main(args=None):
    rclpy.init(args=args)

    node = optimizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
