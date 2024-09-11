#!/usr/bin/env python3
import rclpy
import rclpy.node
import sys, os
import jax
import cvxpy as cp
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from functools import partial
current_dir = os.path.dirname(os.path.abspath(__file__))
# current_dir = os.getcwd()
# sys.path.append('./GPJax')
home_path_op = '/home/colcon_ws/src/online_op/online_op/'
#sys.path.append(current_dir + '/GPJax/')
sys.path.append(home_path_op+'GPJax')
# print(home_path)
# print(sys.path)
jax.config.update("jax_enable_x64", True)
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
from .policy_obstacle import *
# from .test_files.test_policy import policy as policy_test
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
    grad_clip = 1.0 #20
    custom_lr_rate = 0.1
    iter_adam_custom = 4
    trajectory_predictor = None
    violation_factor = 200
    custom_gd_lr_rate_reward = custom_lr_rate / 1                                       # tanh inside parameter vs
    custom_gd_lr_rate_violation = custom_lr_rate * 2
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
        self.height = -0.5 #None
        self.center_x = 0.6 #None
        self.center_y = 0.0 #None
        self.angular_vel = 1.0 #None
        self.trajectory_type_valid = False
        self.horizon = 30 #60 #30 #5

        ###### set up initial drone parameters ######
        self.kx = 7.0
        self.kv = 4.0
        self.kR = 2.0
        self.tanh_factor = 1.0

        self.current_pos = None
        self.current_vel = None
        self.current_state = jnp.zeros(6) #None

        self.pos_ref = None
        self.vel_ref = None
        
        self.ref_valid = False
        self.pose_valid = True


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

        self.num = 0
                
        ###### set up optimizer parameters ######
        # w1 = 0.5
        # w2 = 0.1
        
        # self.op_dt = 0.05
        optimizer.custom_lr_rate = 0.2
        optimizer.custom_gd_lr_rate_reward = optimizer.custom_lr_rate / 20 #2 #1                                       # tanh inside parameter vs
        optimizer.custom_gd_lr_rate_violation = optimizer.custom_lr_rate * 2
        optimizer.grad_clip = 2.0 #1.0 #20.0
        optimizer.iter_adam_custom = 3 #1 #200
        optimizer.violation_factor = 2
        self.optimizer_init = False

        
        self.cp_x = cp.Variable((3,1))
        self.cp_xref = cp.Parameter((3,1))
        self.cp_a = cp.Parameter()
        self.cp_b = cp.Parameter((1,3))
        self.obj = cp.Minimize( self.cp_x.T @ self.cp_xref )
        self.const = [ self.cp_a + self.cp_b @ self.cp_x >= 0 ]
        self.const += [ cp.abs(self.cp_x[0,0])<=10 ]
        self.const += [ cp.abs(self.cp_x[1,0])<=10 ]
        self.const += [ cp.abs(self.cp_x[2,0])<=10 ]
        self.prob = cp.Problem( self.obj, self.const )
        
        ###### set up mavlink ######
        # self.mavlink_ = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        # self.mavlink_ = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
        # self.mavlink_.wait_heartbeat()
        # self.get_logger().info("Mavlink Connected")
        ###### set up publisher for pxhawk ######
        self.publisher_ = self.create_publisher(ParameterReq,'/px4_1/fmu/in/parameter_req',10)
        # self.param_publisher_ = self.create_publisher(ParameterReq,'/px4_1/fmu/in/parameter_req',10)

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
        # self.timer_kx = self.create_timer(0.04, self.kx_callback)
        # self.timer_ky = self.create_timer(0.04, self.kv_callback)
        # self.timer_kR = self.create_timer(0.04, self.kR_callback)
        # self.timer_tanh_factor = self.create_timer(0.04, self.tanh_factor_callback)

        self.timer_parameter_callback = self.create_timer(0.03, self.set_parameter_callback)

        self.timer_optimize = self.create_timer(0.05, self.optimize_callback)

        self.reference_path_publisher = self.create_publisher( Path, '/reference_path', 1)
        self.current_state_publisher = self.create_publisher( PoseStamped, '/current_state', 1)
        self.timer_path = self.create_timer(0.05, self.trajectory_callback)

        

        # # Initialize reward function and its gradient
        # optimizer.get_future_reward = self.setup_reward_func()
        # optimizer.get_future_reward_grad = jit(grad(optimizer   .get_future_reward, argnums=(1)))

        # # Run once to JIT
        # params_policy = jnp.array([self.kx, self.kv])
        # init_state = jnp.array([0.0,0,0,0,0,0]).reshape(-1,1)            
        # optimizer.get_future_reward(init_state, params_policy, jnp.array([0]) )
        # optimizer.get_future_reward_grad( init_state, params_policy, jnp.array([0]) )
        # self.get_logger().info("Gaussian Process Initialized")

        # optimizer.get_future_reward = self.setup_reward_func()
        # optimizer.get_future_reward_grad = jit(grad(optimizer.get_future_reward, argnums=1))
        # deltaT = jnp.array([0])
        # optimizer.get_future_reward( jnp.zeros((6,1)), jnp.array([7.0, 4.0]), 0*deltaT )

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
            optimizer.get_future_reward_grad = jit(jacrev(optimizer.get_future_reward, argnums=(1)))

            

            # Run once to JIT
            params_policy = jnp.array([self.kx, self.kv, self.kR, self.tanh_factor])
            init_state = jnp.array([0.0,0,0,0,0,0]).reshape(-1,1)            
            # optimizer.get_future_reward(init_state, params_policy, jnp.array([0]) )
            params_policy = jnp.array([7, 4.0, 2.0, 1.0])
            # olicy = jnp.array([7,4.0])
            print(optimizer.get_future_reward( init_state, params_policy, jnp.array([0.0]) ))
            print(optimizer.get_future_reward_grad( init_state, params_policy, jnp.array([0.0]) ))
            self.get_logger().info("Gaussian Process Initialized")

            deltaT = jnp.array([(self.get_clock().now().nanoseconds-self.start_time)/10**9])
            # optimizer.get_future_reward_grad( jnp.zeros((6,1)), jnp.array([7.0, 4.0]), 0*deltaT )
            self.get_logger().info(f"grad test {optimizer.get_future_reward_grad( jnp.array([0.,0., 0.0, 0.0, 0, 0]).reshape(-1,1), jnp.array([7.0, 4.0, 2.0, 1.0]), 0*deltaT )}")
            self.get_logger().info(f"grads: {optimizer.get_future_reward_grad( jnp.array([0.38514861, 0.68848354, -0.6646899, 0.07905411, -0.31466046, 0.02618345]).reshape(-1,1), jnp.array([7.0, 4.0, 2.0, 1.0]), jnp.array([45.46159775]) )}")

            # Run once for JIT
            optimizer.trajectory_predictor = self.setup_reference_trajectory_prediction()
            optimizer.trajectory_predictor(deltaT)



            self.trajectory_type_valid = True

    def coordinate_callback(self, msg):
            # if self.ref_valid is False:
            #     self.ref_valid = True
            #     self.initialize_gp()
            self.current_pos = [msg.x, msg.y, msg.z]
            self.current_vel = [msg.vx,msg.vy,msg.vz]
            self.current_state = jnp.array(self.current_pos + self.current_vel)

            self.pose_valid = True
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "vicon/world"
            pose.pose.position.x = self.current_pos[1]
            pose.pose.position.y = self.current_pos[0]
            pose.pose.position.z = -self.current_pos[2]
            yaw = jnp.arctan2( self.current_vel[0], self.current_vel[1] )
            pose.pose.orientation.w = float(jnp.cos( yaw/2 ))
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = float(jnp.sin( yaw/2 ))
            self.current_state_publisher.publish(pose)

            # print(self.current_state)
            # self.get_logger().info(f'callback state is:  {self.current_state}')

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
        # gp_file_path = home_path_op+'gp_models/'
        # gp_file_x = gp_file_path + 'sparsegp_model_x_norm5_clipped_moredata.pkl'
        # gp_file_y = gp_file_path + 'sparsegp_model_y_norm5_clipped_moredata.pkl'
        # gp_file_z = gp_file_path + 'sparsegp_model_z_norm5_clipped_moredata.pkl'
        
        ###### load Datasets ######
        # trainset_file_path = home_path_op+'dataset/'
        # train_x = jnp.load(trainset_file_path + 'training_disturbance_x.npy')
        # train_y = jnp.load(trainset_file_path + 'training_disturbance_y.npy')
        # train_z = jnp.load(trainset_file_path + 'training_disturbance_z.npy')
        # x = jnp.load(trainset_file_path+'training_input.npy')
        # y = jnp.column_stack((train_x, train_y, train_z))


        home_path_op = '/home/colcon_ws/src/online_op/online_op/'
        gp_file_path = home_path_op+'gp_models_new/'
        trainset_file_path = home_path_op+'dataset_new/'
        disturbance_path = trainset_file_path + 'disturbance.npy'
        input_path = trainset_file_path + 'input.npy'

        gp_train_x = jnp.load(input_path)
        gp_train_x = gp_train_x#[::140]
        gp_train_y = jnp.load(disturbance_path)
        gp_train_y = gp_train_y.T#[::140].T

        file_path1 = gp_file_path + 'sparsegp_model_x_norm5_clipped.pkl'
        file_path2 = gp_file_path + 'sparsegp_model_y_norm5_clipped.pkl'
        file_path3 = gp_file_path + 'sparsegp_model_z_norm5_clipped.pkl'

        self.gp0 = initialize_gp_prediction(file_path1)
        self.gp1 = initialize_gp_prediction(file_path2)
        self.gp2 = initialize_gp_prediction(file_path3)

        x = gp_train_x
        y = gp_train_y
        
        # trainset_slice = 50
        # x = x[::trainset_slice]
        # y = y[::trainset_slice].T
        # self.training_state = x
        # self.training_disturbance = y
        
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

        self.get_logger().info('Initialized Gaussian Process Models')


    def setup_reference_trajectory_prediction(self):

        trajectory_type_int = self.trajectory_type_int
        radius = self.radius
        angular_vel = self.angular_vel
        center_x = self.center_x
        center_y = self.center_y
        height = self.height
        opt_dt = 0.05
        horizon = self.horizon

        print(f"INFO: {radius}, {angular_vel}, {center_x}, {center_y}, {height}, {horizon} {self.start_time}")

        @jit
        def func(deltaT):
            reference_states = jnp.zeros((3,horizon))
            reference_yaws = jnp.zeros(horizon)

            @jit
            def body(h, inputs):
                reference_states, reference_yaws = inputs
                t = deltaT + h * opt_dt
                ref_pos,ref_vel,ref_acc = find_ref_pos_vel_acc(trajectory_type_int,t,[radius, angular_vel, center_x, center_y, height])
                reference_states = reference_states.at[:,h].set( ref_pos )
                reference_yaws = reference_yaws.at[h].set( jnp.arctan2( ref_vel[1], ref_vel[0] ) )
                return reference_states, reference_yaws
            reference_states, reference_yaws = lax.fori_loop( 0, horizon, body, (reference_states, reference_yaws) )
            return reference_states, reference_yaws   

        return func     


    def setup_reward_func(self):

        trajectory_type_int = self.trajectory_type_int
        radius = self.radius
        angular_vel = self.angular_vel
        center_x = self.center_x
        center_y = self.center_y
        height = self.height

        horizon = self.horizon
        print(f"INFO :::::::::::::::::::::: horizon: {horizon}")
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
            # jax.debug.print("state optimize: {x}", x=X.T)
            # jax.debug.print("deltaT: {x1}, X:{x2}, policy_params: {x3}, states: {x4}, weights: {x5}", x1=deltaT, x2=X, x3=policy_params, x4=states, x5=weights)
            kx = policy_params[0]
            kv = policy_params[1]
            # print(f"params: {policy_params}")
            w1 = 0.5
            w2 = 0.5
            opt_dt = 0.05
            # reward = 0 + w1 * (kx-7)**2 + w2 * (kv-4)**2
            reward = w1 * (kx)**2 + w2 * (kv)**2
            constraint = 0
            # reward_temp = jnp.sum(jnp.array([w1, w2]) * jnp.square(policy_params))
            # # reward = 0

            # jax.debug.print( "inputs! = {w1}, {w2}, {kx}, {kv}", w1=w1, w2=w2, kx=kx, kv=kv )
            # print( f"inputs! = {reward_temp}, {states}, {weights}" )
            
            # def body_test(i, inputs):
            #     # x = inputs
            #     x = inputs
            #     jax.debug.print("inputs test!!")
            #     # return x
            #     return x #, states, weights
            # # x = lax.fori_loop(0, horizon, body_test, jnp.array([0]))
            # test =  lax.fori_loop( 0, horizon, body_test, reward_temp )
            # # return test
            # return reward_temp

            # reference_positions = jnp.zeros((3,horizon))
            # reference_yaws = jnp.zeros(horizon)
            def body(h, inputs):
                '''
                Performs UT-EC with 6 states
                '''
                t = deltaT + h * opt_dt
                # jax.debug.print("**********************************  inside body ***********************")
                reward, states, weights, constraint = inputs
                ref_pos,ref_vel,ref_acc = find_ref_pos_vel_acc(trajectory_type_int,t,[radius, angular_vel, center_x, center_y, height])
                control_inputs, pos_ref, vel_ref = policy( states, policy_params, [ref_pos,ref_vel,ref_acc])         # mean_position = get_mean( states, weights )
                # control_inputs, pos_ref, vel_ref = policy_test( t[0], states, policy_params )         # mean_position = get_mean( states, weights )
                # jax.debug.print("**************************************1********************************************************")
                next_states_mean, next_states_cov = get_next_states_with_sparse_gp_sigma_inv( states, control_inputs, opt_dt, [gp0, gp1, gp2], [L0, L1, L2], [L0_inv, L1_inv, L2_inv],  [Lz0, Lz1, Lz2], [Lz_inv0, Lz_inv1, Lz_inv2], [Kzz_inv_Kzx_diff0, Kzz_inv_Kzx_diff1, Kzz_inv_Kzx_diff2])
                next_states_expanded, next_weights_expanded = sigma_point_expand_with_mean_cov( next_states_mean, next_states_cov, weights)
                # jax.debug.print("***************************************2*******************************************************")
                next_states, next_weights = sigma_point_compress( next_states_expanded, next_weights_expanded )
                # jax.debug.print("**********************************************************************************************")
                states = next_states
                # jax.debug.print("***************************************3*******************************************************")
                weights = next_weights
                # jax.debug.print("***************************************4*******************************************************")
                # jax.debug.print("control {x}, states {y}", x=control_inputs, y=states)
                # jax.debug.print("***************************************5*******************************************************")
                reward = reward + reward_func( states, weights, pos_ref, vel_ref ) # reward is loss
                constraint = constraint + constraint_violation( states, weights, jnp.array([-0.4, 0.2, -0.5]).reshape(-1,1), 0.4 )
                return reward, states, weights, constraint
            # jax.debug.print("*******************************************10***************************************************")
            reward, _, _,constraint =  lax.fori_loop( 0, horizon, body, (reward, states, weights, constraint) )
            return jnp.array([reward, constraint])
        return compute_reward

    
    def optimize_scipy(self, deltaT):
        # print("Optimizing with Scipy")
        gp_train_x = self.training_state
        
        gp_train_y = self.training_disturbance
        params_policy = jnp.array([self.kx, self.kv])
        init_state = jnp.array(self.current_state)
        ref_pos,ref_vel,ref_acc = self.find_ref_pos_vel_acc(deltaT)


    def optimize_callback(self):
        if not self.optimizer_init:
            return
        if not self.pose_valid:
            return
        if self.trajectory_type_valid is True:
            # self.get_logger().info(f'optimizing')
            deltaT = jnp.array([(self.get_clock().now().nanoseconds-self.start_time)/10**9])

            params_policy = jnp.array([self.kx, self.kv, self.kR, self.tanh_factor])
            init_state = jnp.array(self.current_state).reshape(-1,1)

            # params_policy = jnp.array([self.kx, self.kv])
            # init_state = jnp.zeros((6,1))

            # t0 = self.get_clock().now().nanoseconds
            kx, kv, kR, tanh_factor = self.optimize(init_state, deltaT, params_policy)
            self.kx, self.kv, self.kR, self.tanh_factor = np.clip(kx, 0.01, 30), np.clip(kv, 0.01, 30), np.clip(kR, -30, 30), np.clip(tanh_factor, -30, 30)
            # t1 = self.get_clock().now().nanoseconds
            # self.get_logger().info(f"time taken : {(t1-t0)/10**9}")

    def trajectory_callback(self):
        if self.trajectory_type_valid is True:
           
            deltaT = jnp.array([(self.get_clock().now().nanoseconds-self.start_time)/10**9])
            

            
            t0 = self.get_clock().now().nanoseconds
            predicted_path, predicted_yaws = optimizer.trajectory_predictor( deltaT )
            t1 = self.get_clock().now().nanoseconds
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "vicon/world"
            for i in range(self.horizon):
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(predicted_path[1,i])
                pose.pose.position.y = float(predicted_path[0,i])
                pose.pose.position.z = -float(predicted_path[2,i])
                pose.pose.orientation.w = float(np.sin(predicted_yaws[i]/2))
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = float(np.cos(predicted_yaws[i]/2))
                path.poses.append( pose )
            
            # self.get_logger().info(f"path init: {predicted_path[:,0]}, time: {(t1-t0)/10**9}")
            self.reference_path_publisher.publish( path )


    # @staticmethod
    # @jit
    def optimize(self,init_state, deltaT, params_policy):
        

            for i in range(optimizer.iter_adam_custom):
                reward, const = optimizer.get_future_reward( init_state, params_policy, deltaT )
                self.get_logger().info(f'const: {const}')
                grads = optimizer.get_future_reward_grad( init_state, params_policy, deltaT )
                params_policy_grad = grads[0,:]
                params_policy_grad = jnp.clip( params_policy_grad, -optimizer.grad_clip, optimizer.grad_clip )

                violation_grad = grads[1,:]
                violation_grad = jnp.clip( violation_grad, -optimizer.grad_clip, optimizer.grad_clip )

                # self.cp_xref.value = np.asarray(params_policy_grad).reshape(-1,1)
                # self.cp_a.value = max( np.asarray(const), 0.0)
                # self.cp_b.value = np.asarray(violation_grad).reshape(1,-1)
                # self.prob.solve()

                # params_policy_grad = jnp.clip( self.cp_x.value[:,0], -optimizer.grad_clip, optimizer.grad_clip )
                # params_policy = params_policy + optimizer.custom_lr_rate * params_policy_grad

                if const>=-0.001: # all safe!
                    params_policy_grad = params_policy_grad
                    params_policy = params_policy - optimizer.custom_gd_lr_rate_reward * params_policy_grad
                    params_policy = params_policy.at[3].set(jnp.clip(params_policy[3], 0.001, None))
                    break
                else:
                    # got unsafe!
                    self.get_logger().info(f'const: {const}, violation grad: {violation_grad}')
                    params_policy_grad = jnp.clip( optimizer.violation_factor * violation_grad, -optimizer.grad_clip, optimizer.grad_clip )
                    params_policy = params_policy + optimizer.custom_gd_lr_rate_violation * params_policy_grad # * 2
                params_policy = params_policy.at[3].set(jnp.clip(params_policy[3], 0.001, None))
            self.get_logger().info(f'returned params: {params_policy}')
            op_kx = params_policy[0]
            op_kv = params_policy[1]
            op_kR = params_policy[2]
            op_tanh_factor = params_policy[3]
            return op_kx, op_kv, op_kR, op_tanh_factor
    
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

    def set_parameter_callback(self):
        
        
        # do self.num this time
        if self.num==0:
            message_kx = self.create_ParameterReq_msg('QUAD_KX', round(self.kx,2))
            self.publisher_.publish(message_kx)
            self.get_logger().info(f'X: QUAD_KX is:  {self.kx} and QUAD_KV is: {self.kv} and kR is {self.kR}, and ktanh_factor is {self.tanh_factor}')
            self.num=1
        elif self.num==1:
            message_kv = self.create_ParameterReq_msg('QUAD_KV', round(self.kv,2))
            self.publisher_.publish(message_kv)
            self.num=2
        elif self.num==2:
            message_kR = self.create_ParameterReq_msg('QUAD_OBS_K0', round(self.kR**2,2))
            self.publisher_.publish(message_kR)
            self.num=3
        elif self.num==3:
            message_tanh_factor = self.create_ParameterReq_msg('QUAD_OBS_K1', round(self.tanh_factor**2,2))
            self.publisher_.publish(message_tanh_factor)
            self.num=0
        # self.num = self.num + 1

        
    # def kx_callback(self):
    #     message_kx = self.create_ParameterReq_msg('QUAD_KX', round(self.kx,2))
    #     self.publisher_.publish(message_kx)
    #     self.get_logger().info(f'X: QUAD_KX is:  {self.kx} and QUAD_KV is: {self.kv} and kR is {self.kR}, and ktanh_factor is {self.tanh_factor}')

    # def kv_callback(self):
    #     message_kv = self.create_ParameterReq_msg('QUAD_KV', round(self.kv,2))
    #     self.publisher_.publish(message_kv)
    #     # self.get_logger().info(f'Y: QUAD_KX is:  {self.kx} and QUAD_KV is: {self.kv}')

    # def kR_callback(self):
    #     message_kR = self.create_ParameterReq_msg('QUAD_OBS_K0', round(self.kR**2,2))
    #     self.publisher_.publish(message_kR)
    #     # return

    # def tanh_factor_callback(self):
    #     message_tanh_factor = self.create_ParameterReq_msg('QUAD_OBS_K1', round(self.tanh_factor**2,2))
    #     self.publisher_.publish(message_tanh_factor)
        # return 

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
