#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleAttitude, ParameterReq, ParameterRes
from foresee_msgs.msg import DynamicsData as CombinedData
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.spatial.transform import Rotation as R
class message(Node):
    def __init__(self):
        super().__init__('message')

        qos_profile = QoSProfile(
                            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                            depth=1
        )
        
        ###### set up parameters ######
        self.radius = 0.2
        self.height = -0.4
        self.center_x = 0.0
        self.center_y = 0.0
        self.angular_vel = 1.0
        self.ned_pos = None
        self.ned_vel = None
        self.ned_acc = None
        self.pos_ref = None
        self.vel_ref = None
        self.acc_ref = None
        self.acc_com = None
        self.angles = None
        self.quaternion = None
        self.kx = 7.0#None
        self.kv = 4.0#None
        self.ref_valid = False
        self.angle_valid = False
        self.gains_valid = False
        ############ set up publisher ############
        self.publisher_ = self.create_publisher(CombinedData, '/drone/combined_data', 10)
        self.parameter_req_publisher_ = self.create_publisher(ParameterReq,'/px4_1/fmu/in/parameter_req',10)
        ############ set up timer for gains ############
        self.timer = self.create_timer(1.0/80.0, self.timer_callback)
        ################## set up Subscription ##################

        #self.timer = self.create_timer(1./80., self.timer_callback)
        self.actual = self.create_subscription(
		    VehicleLocalPosition,
		    '/px4_1/fmu/out/vehicle_local_position',
		    self.coordinate_callback,
		    #10,
            qos_profile=qos_profile)
        
        self.ref = self.create_subscription(
		    TrajectorySetpoint,
		    '/px4_1/fmu/in/trajectory_setpoint',
		    self.reference_callback,
		    10)
        self.angle = self.create_subscription(
		    VehicleAttitude,
		    '/px4_1/fmu/out/vehicle_attitude',
		    self.angle_callback,
		    # 10)
            qos_profile=qos_profile)
        self.gains_valid = True
        # self.gains = self.create_subscription(
		#     ParameterRes,
		#     '/px4_1/fmu/out/parameter_res',
		#     self.gains_callback,
		#     # 10)
        #     qos_profile=qos_profile)
        
    def timer_callback(self):
        self.request_gains()
    def create_CombinedData_msg(self):
        msg = CombinedData()
        msg.pos = self.ned_pos
        msg.vel = self.ned_vel
        msg.acc = self.ned_acc
        msg.pos_ref = self.pos_ref
        msg.vel_ref = self.vel_ref
        msg.acc_ref = self.acc_ref
        msg.angles = self.angles
        msg.quaternion = self.quaternion
        msg.kx = self.kx
        msg.kv = self.kv
        return msg
        ################## set up call backs ##################
    

    def create_Parameter_Req_msg(self, param_name_, value_=0):
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
        msg.set = False
        # print("value_ is: ", value_)
        # print("Type of value is: ", type(value_.item()))
        msg.value = float(value_)
        return msg
    
    def request_gains(self):
        kx_msg = self.create_Parameter_Req_msg('QUAD_KX')
        kv_msg = self.create_Parameter_Req_msg('QUAD_KV')
        self.parameter_req_publisher_.publish(kx_msg)
        self.parameter_req_publisher_.publish(kv_msg)

    def coordinate_callback(self, msg):
        self.ned_pos = [msg.x,msg.y,msg.z]
        self.ned_vel = [msg.vx,msg.vy,msg.vz]
        self.ned_acc = [msg.ax, msg.ay, msg.az]
        
        if not self.ref_valid or not self.angle_valid or not self.gains_valid:
            return
        message = self.create_CombinedData_msg()
        self.publisher_.publish(message)
             
    def reference_callback(self, msg):
        self.pos_ref = msg.position
        self.vel_ref = msg.velocity
        self.acc_ref = msg.acceleration
        self.ref_valid = True
    
    
    def angle_callback(self, msg):
        
        quat = msg.q
        w, x, y, z = quat
        r =  R.from_quat([w,x,y,z], scalar_first = True)
        roll, pitch, yaw = r.as_euler('xyz',degrees=True)
        self.angles = [roll, pitch, yaw]
        self.quaternion = [float(w),float(x),float(y),float(z)]
        self.angle_valid = True
    
    # def gains_callback(self, msg):
    #     param_name = msg.param_name
    #     if param_name == 'QUAD_KX':
    #         self.kx = msg.value
    #     if param_name == 'QUAD_KV':
    #         self.kv = msg.value
    #     if self.kx is not None and self.kv is not None:
    #         self.gains_valid = True
def main(args=None):
    rclpy.init(args=args)

    node = message()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
