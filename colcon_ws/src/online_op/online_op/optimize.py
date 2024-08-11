#!/usr/bin/env python3
import rclpy
import rclpy.node
import sys, os
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
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from .test_jax_utils import *
from .test_gp_utils import *
from .test_policy import *
from foresee_msgs.msg import TrajectoryInfo
from pymavlink import mavutil
from .optimize_helper import *
from jax import grad, jit
os.environ['JAX_TRACEBACK_FILTERING'] = 'off'
# import pymavparam as pm

class optimizer(Node):
    def __init__(self):
        super().__init__('optimizer')

        # qos_profile = QoSProfile(
        #                     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        #                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #                     depth=1
        # )

        
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
        self.radius = None
        self.height = None
        self.center_x = None
        self.center_y = None
        self.angular_vel = None
        self.trajectory_type_valid = False

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
        self.get_logger().info("Gaussian Process Initialized")
        
        ###### set up optimizer parameters ######
        # w1 = 0.5
        # w2 = 0.1
        self.horizon = 100
        self.op_dt = 0.05
        self.custom_lr_rate = 0.1
        self.grad_clip = 1.0
        self.iter_adam_custom = 200
        
        ###### set up mavlink ######
        # self.mavlink_ = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        # self.mavlink_ = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
        # self.mavlink_.wait_heartbeat()
        self.get_logger().info("Mavlink Connected")

        
        ################## set up Subscription ##################
        self.drone_coordinates = self.create_subscription(
		    VehicleLocalPosition,
		    '/px4_1/fmu/out/vehicle_local_position',
		    self.coordinate_callback,
		    10)
            #qos_profile=qos_profile)
        
        self.trajectory_info = self.create_subscription(
		    TrajectoryInfo,
		    '/drone/TrajectoryInfo',
		    self.trajectory_info_callback,
		    10)
        
    def trajectory_info_callback(self,msg):
        if self.trajectory_type_valid is False:
            self.trajectory_type = msg.type
            self.radius = msg.radius
            self.angular_vel = msg.angular_vel
            self.center_x = msg.center_x
            self.center_y = msg.center_y
            self.start_time = msg.start_time
            self.trajectory_type_valid = True

    def coordinate_callback(self, msg):
            # if self.ref_valid is False:
            #     self.ref_valid = True
            #     self.initialize_gp()
            self.current_pos = [msg.x, msg.y, msg.z]
            self.current_vel = [msg.vx,msg.vy,msg.vz]
            self.current_state = jnp.array(self.current_pos + self.current_vel)
            # print(self.current_state.shape)
            
            if self.trajectory_type_valid is True:
                deltaT = (self.get_clock().now().nanoseconds-self.start_time)/10**9
                # ref_coord = self.find_ref_coord(deltaT)
                self.kx, self.kv = self.optimize(deltaT)
                self.publish_optimal_gains()




    def initialize_gp(self):
        self.get_logger().info('Initializing Gaussian Process Models')
        ###### load gaussian process models ######
        gp_file_path = home_path_op+'gp_models/'
        gp_file_x = gp_file_path + 'gp_model_x_norm5_clipped.pkl'
        gp_file_y = gp_file_path + 'gp_model_y_norm5_clipped.pkl'
        gp_file_z = gp_file_path + 'gp_model_z_norm5_clipped.pkl'
        self.gp0 = initialize_gp_prediction(gp_file_x)
        self.gp1 = initialize_gp_prediction(gp_file_y)
        self.gp2 = initialize_gp_prediction(gp_file_z)
        
        ###### load Datasets ######
        trainset_file_path = home_path_op+'dataset/'
        train_x = jnp.load(trainset_file_path + 'training_disturbance_x.npy')
        train_y = jnp.load(trainset_file_path + 'training_disturbance_y.npy')
        train_z = jnp.load(trainset_file_path + 'training_disturbance_z.npy')
        x = jnp.load(trainset_file_path+'training_input.npy')
        y = jnp.column_stack((train_x, train_y, train_z))
        
        trainset_slice = 10
        x = x[::trainset_slice]
        y = y[::trainset_slice].T
        self.training_state = x
        self.training_disturbance = y
        
        D0 = gpx.Dataset(X=x, y=y[0].reshape(-1,1))
        D1 = gpx.Dataset(X=x, y=y[1].reshape(-1,1))
        D2 = gpx.Dataset(X=x, y=y[2].reshape(-1,1))
        ###### compute the inverses ######
        self.sigma0 = self.gp0.compute_sigma_inv(train_data=D0)
        self.sigma1 = self.gp1.compute_sigma_inv(train_data=D1)
        self.sigma2 = self.gp2.compute_sigma_inv(train_data=D2)

    

    def optimize(self,deltaT):
        
        print("Optimizing")
        gp_train_x = self.training_state
        
        gp_train_y = self.training_disturbance
        params_policy = jnp.array([self.kx, self.kv])
        init_state = jnp.array(self.current_pos)
        print("initial state type is: ",type(init_state))
        print("policy params type is: ",type(params_policy))
        print("gp train type is: ",type(gp_train_x), type(gp_train_y))
        print("deltaT type is: ",type(deltaT))
        def body(i, inputs):
            params_policy = inputs
            params_policy_grad = get_future_reward_grad( init_state, params_policy, gp_train_x, gp_train_y, deltaT)
            params_policy_grad = jnp.clip( params_policy_grad, -self.grad_clip, self.grad_clip )
            params_policy = params_policy - self.custom_lr_rate * params_policy_grad
            return params_policy
        
        # print(type(self.kx), type(self.kv))
        params_policy = lax.fori_loop(0, self.iter_adam_custom, body, params_policy)
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



    def publish_optimal_gains(self):
        self.get_logger().info(f'Sending Gains: QUAD_KX = {self.kx}, QUAD_KV = {self.kv}')
        self.mavlink_.mav.param_set_send(
            self.mavlink_.target_system, self.mavlink_.target_component,
            b'QUAD_KX',
            self.kx,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        self.mavlink_.mav.param_set_send(
            self.mavlink_.target_system, self.mavlink_.target_component,
            b'QUAD_KV',
            self.kv,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

def main(args=None):
    rclpy.init(args=args)

    node = optimizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
