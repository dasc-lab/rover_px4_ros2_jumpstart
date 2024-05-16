import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from px4_msgs.msg import TrajectorySetpoint
from foresee_msgs.msg import DynamicsData as CombinedData
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import time



class data(Node):
    def __init__(self):
        super().__init__('data')
         ###### set up parameters ######
        self.radius = 0.2
        self.height = -0.4
        self.center_x = 0.0
        self.center_y = 0.0
        self.angular_vel = 1.0
        self.g = 9.81
        self.ned_pos = None
        self.ned_vel = None
        self.ned_acc = None
        self.pos_ref = None
        self.vel_ref = None
        self.acc_ref = None
        self.kv = 7.4
        self.kx = 14
        self.m = 0.681
        self.thrust = None
        self.acc_cmd_arr = []
        self.acc_ned_arr = []
        self.start = time.time()
        self.acc_diffs = []
        self.data = self.create_subscription(
		    CombinedData,
		    '/drone/combined_data',
		    self.data_callback,
		    10)
    def data_callback(self, msg):
        self.ned_pos = msg.pos
        self.ned_vel = msg.vel
        self.ned_acc = msg.acc
        self.pos_ref = msg.pos_ref
        self.vel_ref = msg.vel_ref
        self.acc_ref = msg.acc_ref
        self.calculate_thrust()
        self.acc_cmd_arr.append(self.thrust/self.m)
        self.acc_ned_arr.append(self.ned_acc)
        acc_diff = self.ned_acc - self.acc_com
        self.acc_diffs.append(acc_diff)
        print(f"data callback")
        if(time.time() - self.start) > 25:
            print(f"save and break")
            np.save("acc_cmd", self.acc_cmd_arr)
            np.save("acc_ned", self.acc_ned_arr)
            np.save('disturbance',self.acc_diffs)
            print("length:", len(self.acc_diffs))
            rclpy.shutdown()
            node.destroy_node()
    def calculate_thrust(self):
        diff_pos = np.array(self.ned_pos - self.pos_ref)
        diff_vel = np.array(self.ned_vel - self.vel_ref)
        diff_acc = np.array(self.ned_acc - self.acc_ref)
        e3 = np.array([0,0,1.]).reshape(diff_pos.shape)
        # 3D vector thrust
        self.thrust = -self.kx * diff_pos - self.kv * diff_vel + self.m * self.acc_ref
        #- self.m * self.g * e3 
        self.acc_com = self.thrust/self.m

def main(args=None):
    rclpy.init(args=args)
    node = data()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
