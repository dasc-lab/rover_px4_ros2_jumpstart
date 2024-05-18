#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.clock import Clock
import time


class driveCircle(Node):
    def __init__(self):
        super().__init__('driveCircle')

        ###### set up circle parameters ######
        self.radius = 0.2
        self.height = -0.4
        self.center_x = 0.0
        self.center_y = 0.0
        self.home_x = None
        self.home_y = None
        self.angular_vel = 1.0
        #self.initialized = False
        self.program_start = time.time()
        ###### set up node parameters ######
        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.coordinate = None
        self.quat = None
        self.world_coordinate = None
        self.clock  = self.get_clock()
        self.start_time = self.get_clock().now().nanoseconds
        #self.dt = 0.05

        ################## set up Subscription ##################
        self.timer = self.create_timer(1./80., self.timer_callback)
        self.coord_sub = self.create_subscription(
		    VehicleLocalPosition,
		    '/px4_1/fmu/out/vehicle_local_position',
		    self.coordinate_callback,
		    10)
            #qos_profile=qos_profile)
    
    #def get_ground_truth_coord(self):
    def coordinate_callback(self,msg):
        if self.home_x is None and self.home_y is None:
            self.home_x = msg.x
            self.home_y = msg.y
        self.coordinate = [msg.x,msg.y,msg.z]
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
    
    def create_TrajectorySetpoint_msg(self):
        ''' Create message in NED frame '''
        msg = TrajectorySetpoint()
        
        waypoint = self.calculate_waypoint()
        vel_ref = self.calculate_vel_ref()
        acc_ref = self.calculate_acc_ref()

        msg.position[0] = waypoint[0] #world_coordinates[0]
        msg.position[1] = waypoint[1] #world_coordinates[1]
        msg.position[2] = self.height #world_coordinates[2]
        msg.yaw = 290 * 3.14/180.0 #0.0
        for i in range(3):
            msg.velocity[i] = vel_ref[i]
            msg.acceleration[i] = acc_ref[i]
         
        msg.jerk[0] = msg.jerk[1] = msg.jerk[2] = 0 
        msg.yawspeed = 0.0
        return msg
    def calculate_intermediate_waypoint(self, waypoint):
        '''
        calculate intermediate waypoint in case drone is too far away
        '''
        def calculate_unit_vector(point1, point2):
            """
            Calculate the unit vector from point1 to point2.

            Parameters:
            point1 (list or tuple): Coordinates of the first point (x1, y1, z1).
            point2 (list or tuple): Coordinates of the second point (x2, y2, z2).

            Returns:
            numpy.ndarray: The unit vector from point1 to point2.
            """
            
            point1 = np.array(point1)
            point2 = np.array(point2)

            
            vector = point2 - point1

            # Calculate the magnitude of the vector
            magnitude = np.linalg.norm(vector)

            # Calculate the unit vector
            unit_vector = vector / magnitude

            return unit_vector
        unit_vector = calculate_unit_vector(self.coordinate, waypoint)
        intermediate = 0.2 * unit_vector
        return intermediate
    def create_hover_TrajectorySetpoint_msg(self, waypoint):
        msg = TrajectorySetpoint()
        msg.position[0] = waypoint[0] #world_coordinates[0]
        msg.position[1] = waypoint[1] #world_coordinates[1]
        msg.position[2] = self.height #world_coordinates[2]
        msg.yaw = 290 * 3.14/180.0 #0.0
        for i in range(3):
            msg.velocity[i] = 0
            msg.acceleration[i] = 0
         
        msg.jerk[0] = msg.jerk[1] = msg.jerk[2] = 0 
        msg.yawspeed = 0.0
        return msg
    def create_TrajectorySetpoint_msg_intermediate(self,waypoint):
        msg = TrajectorySetpoint()
        intermediate = self.calculate_intermediate_waypoint(waypoint)
        assert intermediate[2] < 0
        
        msg.position[0] = intermediate[0] # world_coordinates[0]
        msg.position[1] = intermediate[1] # world_coordinates[1]
        msg.position[2] = self.height # world_coordinates[2]
        # msg.yaw = (3.1415926 / 180.) * (float)(setpoint_yaw->value())
        msg.yaw = 290 * 3.14/180.0 #0.0
        for i in range(3):
            msg.velocity[i] = 0.0
            msg.acceleration[i] = 0.0
            msg.jerk[i] = 0.0
        #msg.velocity = [0.2, 0.2, 0.2]
        msg.yawspeed = 0.0
        return msg
    
    def timer_callback(self):
        
        def euclidean_distance(point1, waypoint):
            distance = np.sqrt((waypoint[0] - point1[0])**2 + (waypoint[1] - point1[1])**2 + (waypoint[2] - point1[2])**2)
            return distance
        
        if time.time() - self.program_start < 5: 
            msg = self.create_hover_TrajectorySetpoint_msg([self.home_x, self.home_y, self.height])
            self.publisher_.publish(msg)
            return
        waypoint = self.calculate_waypoint()
        msg = self.create_TrajectorySetpoint_msg()

        if euclidean_distance(self.coordinate, waypoint)>0.02 and self.coordinate is not None:
            msg = self.create_TrajectorySetpoint_msg_intermediate(waypoint)


        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    node = driveCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
