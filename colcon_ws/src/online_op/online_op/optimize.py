#!/usr/bin/env python3
import rclpy
import rclpy.node
import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
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
        
        self.declare_parameter('trajectory_type', 'circle')
        self.trajectory_type = self.get_parameter('trajectory_type').get_parameter_value().string_value
        self.get_logger().info(f'Trajectory type: {self.trajectory_type}')
        self.radius = 0.4
        self.height = -0.5
        self.center_x = 0.6
        self.center_y = 0.0
        self.angular_vel = 1.0


        ###### set up initial parameters ######
        self.kx = 7
        self.kv = 4

        self.ned_pos = None
        self.ned_vel = None
        self.ned_acc = None
        self.pos_ref = None
        self.vel_ref = None
        self.acc_ref = None
        self.acc_com = None
        self.ref_valid = False
        ###### set up node parameters ######
        
        self.gp0, self.gp1, self.gp2 = None
        self.clock  = self.get_clock()
        self.start_time = self.get_clock().now().nanoseconds


        ###### set up optimizer parameters ######
        self.op_horizon = 50
        self.op_dt = 0.05
        
        ################## set up Subscription ##################
        self.drone_coordinates = self.create_subscription(
		    VehicleLocalPosition,
		    '/px4_1/fmu/out/vehicle_local_position',
		    self.coordinate_callback,
		    10)
            #qos_profile=qos_profile)

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
        trainset_file_path = 'dataset/'
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

    def coordinate_callback(self):
        if self.ref_valid is False:
            self.ref_valid = True
            self.initialize_gp()
            
        deltaT = (self.get_clock().now().nanoseconds-self.start_time)/10**9
        ref_coord = self.find_ref_coord(deltaT)
        self.kx, self.kv = self.optimizer(ref_coord)
        self.publish_optimal_gains()

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


def main(args=None):
    rclpy.init(args=args)

    node = optimize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
