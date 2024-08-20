#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from px4_msgs.msg import TrajectorySetpoint
from foresee_msgs.msg import TrajectoryInfo
from rclpy.clock import Clock


class driveCircle(Node):
    def __init__(self):
        super().__init__('driveCircle')

        ###### set up circle parameters ######
        self.radius = 0.4
        self.height = -0.5
        self.center_x = 0.6
        self.center_y = 0.0
        self.angular_vel = 1.0

        ###### set up node parameters ######
        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.trajectory_info_publisher_ = self.create_publisher(TrajectoryInfo,'/drone/TrajectoryInfo',10)
        self.coordinate = None
        self.quat = None
        self.world_coordinate = None
        self.clock  = self.get_clock()
        self.start_time = self.get_clock().now().nanoseconds
        #self.dt = 0.05
        # print(f"*************************************************   INFO: {self.radius}, {self.angular_vel}, {self.center_x}, {self.center_y}, {self.height}")

        ################## set up Subscription ##################
        # self.timer = self.create_timer(1./80., self.timer_callback)
        self.timer = self.create_timer(1./50., self.timer_callback)
    
    #def get_ground_truth_coord(self):
    
    def calculate_waypoint(self):
        deltaT = (self.get_clock().now().nanoseconds-self.start_time)/10**9
        x = self.radius * np.cos(self.angular_vel * deltaT) + self.center_x
        y = self.radius * np.sin(self.angular_vel * deltaT) + self.center_y
        waypoint = [y,  x, self.height]
        return waypoint
    
    def calculate_vel_ref(self):
        deltaT = (self.get_clock().now().nanoseconds-self.start_time)/10**9
        vx = -self.radius * self.angular_vel* np.sin(self.angular_vel * deltaT)
        vy = self.radius * self.angular_vel* np.cos(self.angular_vel * deltaT)
        vel_ref = [vy,vx,0]
        return vel_ref
    
    def calculate_acc_ref(self):
        deltaT = (self.get_clock().now().nanoseconds-self.start_time)/10**9
        ax = -self.radius * (self.angular_vel**2) * np.cos(self.angular_vel * deltaT)
        ay = -self.radius * (self.angular_vel**2) * np.sin(self.angular_vel * deltaT)
        acc_ref = [ay,ax,0]
        return acc_ref
    
    def create_trajectory_info_msg(self):
        msg = TrajectoryInfo()
        msg.type = 'circle'
        msg.radius = self.radius
        msg.angular_vel = self.angular_vel
        msg.center_x = self.center_x
        msg.center_y = self.center_y
        msg.start_time = self.start_time
        msg.height = self.height
        return msg
    
    def create_TrajectorySetpoint_msg(self):
        ''' Create message in NED frame '''
        msg = TrajectorySetpoint()
        
        waypoint = self.calculate_waypoint()
        vel_ref = self.calculate_vel_ref()
        acc_ref = self.calculate_acc_ref()

        msg.position[0] = waypoint[0] #world_coordinates[0]
        msg.position[1] = waypoint[1]#world_coordinates[1]
        msg.position[2] = self.height #world_coordinates[2]
        msg.yaw = 0 * 3.14/180.0 #0.0
        for i in range(3):
            msg.velocity[i] = vel_ref[i]
            msg.acceleration[i] = acc_ref[i]
         
        msg.jerk[0] = msg.jerk[1] = msg.jerk[2] = 0 
        msg.yawspeed = 0.0
        return msg

    def create_TrajectorySetpoint_msg_defualt(self):
        msg = TrajectorySetpoint()
        waypoint = self.calculate_waypoint()
        msg.position[0] = waypoint[0] #world_coordinates[0]
        msg.position[1] = waypoint[1]#world_coordinates[1]
        msg.position[2] = self.height #world_coordinates[2]
        #msg.yaw = (3.1415926 / 180.) * (float)(setpoint_yaw->value())
        msg.yaw = 0 * 3.14/180.0 #0.0
        for i in range(3):
            msg.velocity[i] = 0.0
            msg.acceleration[i] = 0.0
            msg.jerk[i] = 0.0
        #msg.velocity = [0.2, 0.2, 0.2]
        msg.yawspeed = 0.0
        return msg
    
    def timer_callback(self):
        msg = self.create_TrajectorySetpoint_msg()
        self.publisher_.publish(msg)
        
        trajectory_info_msg = self.create_trajectory_info_msg()
        self.trajectory_info_publisher_.publish(trajectory_info_msg)

        # self.get_logger().info(f"hello {self.radius}, {self.angular_vel}, {self.center_x}, {self.center_y}, {self.height} {self.start_time}")
        # self.get_logger().info(f"{msg.position[0]}, {msg.position[1]}, {msg.position[2]}")
        
def main(args=None):
    rclpy.init(args=args)

    node = driveCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
