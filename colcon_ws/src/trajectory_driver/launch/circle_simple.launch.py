from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    rosbag = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', 'cir_traj_r0.3_w1.5_c0.40_h0.4_fanhigh',
                '/drone/combined_data'
            ],
            output='screen'
        )
    delayed_recording = TimerAction(
        period = 3.0, 
        actions=[rosbag]
    )
    return LaunchDescription([
        
        Node(
            package='trajectory_driver',  
            executable='circle_simple',  # This should match the name in setup.py
            name='driveCircle', # name of the node
            output='screen'
        ),
        Node(
            package='trajectory_driver', 
            executable='message_node',  
            name='message_node',
            output='screen',
            # parameters=[
            #     {'radius': 0.2},
            #     {'height': -0.4},
            #     {'center_x': 0.0},
            #     {'center_y': 0.0},
            #     {'angular_vel': 1.0},
            # ]
        ),
        Node(
            package='trajectory_driver',  
            executable='data_node',  
            name='data',
            output='screen',
            # parameters=[
            #     {'radius': 0.2},
            #     {'height': -0.4},
            #     {'center_x': 0.0},
            #     {'center_y': 0.0},
            #     {'angular_vel': 1.0},
            #     {'g': 9.81},
            #     {'kv': 7.4},
            #     {'kx': 14},
            #     {'m': 0.681},
            # ]
        ),
        
        delayed_recording
    ])
