import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from foresee_msgs.msg import DynamicsData as CombinedData
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class trajecotry(Node):
    def __init__(self):
        super().__init__('trajectory')

        # qos_profile = QoSProfile(
        #                     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        #                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #                     depth=1
        # )
        
        self.ned_pos = None
        self.ned_vel = None
        self.ned_acc = None
        
        ############ set up publisher ############
        self.publisher_ = self.create_publisher(VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position', 10)
        
        
        ################## set up Subscription ##################

        #self.timer = self.create_timer(1./80., self.timer_callback)
        self.trajectory_data = self.create_subscription(
		    CombinedData,
		    '/drone/combined_data',
		    self.coordinate_callback,
		    10)
            # qos_profile=qos_profile)
        
        # self.ref = self.create_subscription(
		#     TrajectorySetpoint,
		#     '/px4_1/fmu/in/trajectory_setpoint',
		#     self.reference_callback,
		#     10)
    def coordinate_callback(self, msg):
        self.ned_pos = msg.pos
        self.ned_vel = msg.vel
        self.ned_acc = msg.acc
        message = self.create_vehicle_msg()
        self.publisher_.publish(message)
    def create_vehicle_msg(self):
        msg = VehicleLocalPosition()
        msg.x, msg.y, msg.z = self.ned_pos
        msg.vx, msg.vy, msg.vz = self.ned_vel
        msg.ax, msg.ay, msg.az = self.ned_acc

        return msg
    
def main(args=None):
    rclpy.init(args=args)

    node = trajecotry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()