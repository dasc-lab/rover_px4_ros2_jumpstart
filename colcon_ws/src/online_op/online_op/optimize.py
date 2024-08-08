#!/usr/bin/env python3
import rclpy
import rclpy.node
import sys, os
# current_dir = os.path.dirname(os.path.abspath(__file__))
current_dir = os.getcwd()
sys.path.append('./GPJax')
import gpJax as gpx
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from test_jax_utils import *
from test_gp_utils import *
from test_policy import *
from foresee_msgs.msg import TrajectoryInfo
from pymavlink import mavutil
# import pymavparam as pm
class optimize(Node):
    def __init__(self):
        super().__init__('optimize')

        # qos_profile = QoSProfile(
        #                     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        #                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #                     depth=1
        # )

        
        ###### set up trajectory parameters ######
        self.get_logger().info('Optimizer Node Starts')
        self.mavlink_ = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.mavlink_.wait_heartbeat()


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
        ###### set up initial parameters ######
        self.kx = 7
        self.kv = 4

        self.current_pos = None
        self.current_vel = None
        
        self.pos_ref = None
        self.vel_ref = None
        
        self.ref_valid = False
        ###### set up node parameters ######
        
        self.gp0, self.gp1, self.gp2 = None
        self.clock  = self.get_clock()
        self.start_time = self.get_clock().now().nanoseconds


        ###### set up optimizer parameters ######
        self.op_horizon = 50
        self.op_dt = 0.05
        self.custom_lr_rate = 0.1

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
            self.center_y = msg.centery_y
            self.trajectory_type_valid = True

    def coordinate_callback(self, msg):
            if self.ref_valid is False:
                self.ref_valid = True
                self.initialize_gp()
            self.current_pos = [msg.x, msg.y, msg.z]
            if self.trajectory_type_valid is True:
                deltaT = (self.get_clock().now().nanoseconds-self.start_time)/10**9
                ref_coord = self.find_ref_coord(deltaT)
                self.kx, self.kv = self.optimizer(ref_coord)
                self.publish_optimal_gains()



    def initialize_gp(self):
        ###### load gaussian process models ######
        gp_file_path = current_dir+'gp_models/'
        gp_file_x = gp_file_path + 'gp_model_x_norm5_clipped.pkl'
        gp_file_y = gp_file_path + 'gp_model_y_norm5_clipped.pkl'
        gp_file_z = gp_file_path + 'gp_model_z_norm5_clipped.pkl'
        self.gp0 = initialize_gp_prediction(gp_file_x)
        self.gp1 = initialize_gp_prediction(gp_file_y)
        self.gp2 = initialize_gp_prediction(gp_file_z)
        
        ###### load Datasets ######
        trainset_file_path = current_dir+'dataset/'
        train_x = np.load(trainset_file_path + 'training_disturbance_x.npy')
        train_y = np.load(trainset_file_path + 'training_disturbance_y.npy')
        train_z = np.load(trainset_file_path + 'training_disturbance_z.npy')
        x = np.load(trainset_file_path+'training_input.npy')
        y = np.column_stack((train_x, train_y, train_z))

        D0 = gpx.Dataset(X=x, y=y[0].reshape(-1,1))
        D1 = gpx.Dataset(X=x, y=y[1].reshape(-1,1))
        D2 = gpx.Dataset(X=x, y=y[2].reshape(-1,1))
        ###### compute the inverses ######
        self.sigma0 = self.gp0.compute_sigma_inv(train_data=D0)
        self.sigma1 = self.gp1.compute_sigma_inv(train_data=D1)
        self.sigma2 = self.gp2.compute_sigma_inv(train_data=D2)

    

    def optimizer(self, ref_coord):
        
        return op_kx, op_kv
    def find_ref_coord(self, deltaT):
        if self.trajectory_type == 'circle':
            pos_vel_acc = circle_pos_vel_acc
        else:
            pos_vel_acc = figure8_pos_vel_acc
        ref_coord,_,_ = pos_vel_acc(deltaT, self.radius, self. angular_vel, self.center_x, self.center_y)
        return ref_coord.reshape(-1,1)


    def publish_optimal_gains(self):
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

    node = optimize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
