#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from px4_msgs.msg import TrajectorySetpoint
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class optimize(Node):
    def __init__(self):
        super().__init__('optimize')

        qos_profile = QoSProfile(
                            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                            depth=1
        )
        ###### set up initial parameters ######
        self.kx = 7
        self.kv = 4

        ###### set up node parameters ######
        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/', 10)
        self.coordinate = None
        self.quat = None
        self.world_coordinate = None
        self.clock  = self.get_clock()
        self.start_time = self.get_clock().now().nanoseconds
        #self.dt = 0.05
        self.pos = []
        self.velocity = []
        ################## set up Subscription ##################
        self.timer = self.create_timer(1./80., self.timer_callback)
    def coordinate_callback(self):


    def optimizer(self):
        


def main(args=None):
    rclpy.init(args=args)

    node = optimize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
