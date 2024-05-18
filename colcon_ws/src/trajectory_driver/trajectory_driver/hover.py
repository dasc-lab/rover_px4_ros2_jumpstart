#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
import time
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
class hover(Node):
    def __init__(self):
        super().__init__('hover')
        qos_profile = QoSProfile(
                            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                            depth=1
        )
        ###### set up waypoint parameters ######
        self.height = -0.4
        self.center_x = 0.0
        self.center_y = 0.0
        self.initialized = False
        


        ###### set up node parameters ######
        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.coordinate = None
        self.heading = 0.0
        self.start_time = time.time()
        ################## set up Timer ##################
        self.timer = self.create_timer(1./30., self.timer_callback)
        
        ################## Subscribe to Topics ##################
        self.actual = self.create_subscription(
		    VehicleLocalPosition,
		    '/px4_1/fmu/out/vehicle_local_position',
		    self.coordinate_callback,
		    #10,
            qos_profile=qos_profile)
    def coordinate_callback(self,msg):
         if self.initialized is False:
              self.initialized = True
              self.center_x = msg.x
              self.center_y = msg.y
              self.heading = msg.heading
    def create_TrajectorySetpoint_msg(self):
        msg = TrajectorySetpoint()
        msg.position[0] = self.center_x #world_coordinates[0]
        msg.position[1] = self.center_y #world_coordinates[1]
        msg.position[2] = self.height #world_coordinates[2]
        #msg.yaw = (3.1415926 / 180.) * (float)(setpoint_yaw->value())
        msg.yaw = self.heading 
        for i in range(3):
                msg.velocity[i] = 0.0
                msg.acceleration[i] = 0.0
                msg.jerk[i] = 0.0
        #msg.velocity = [0.2, 0.2, 0.2]
        msg.yawspeed = 0.0
        return msg
    
    def timer_callback(self):
        if time.time() - self.start_time > 5:
             rclpy.shutdown()
        if self.initialized : 
            msg = self.create_TrajectorySetpoint_msg()
            self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    node = hover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
