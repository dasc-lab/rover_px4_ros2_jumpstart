
from launch import LaunchDescription
from launch.actions import TimerAction,ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    
    hover_node = Node(
        package='trajectory_driver',
        executable='hover',
        name='hover',
        output='screen'
    )

    
    circle_node = Node(
        package='trajectory_driver',
        executable='circle_simple',  # This should match the name in setup.py
        name='driveCircle',  # name of the node
        output='screen'
    )

    message_node = Node(
        package='trajectory_driver',
        executable='message_node',
        name='message_node',
        output='screen'
    )

    data_node = Node(
        package='trajectory_driver',
        executable='data_node',
        name='data',
        output='screen'
    )

    # Define the rosbag record process
    rosbag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', 'cir_traj_r0.4_w2_c00_h0.4_fanhigh',
            '/drone/combined_data'
        ],
        output='screen'
    )

    # Define the delayed recording action
    delayed_recording = TimerAction(
        period=3.0,
        actions=[rosbag]
    )

    # Event handler to start other nodes when 'hover' node exits
    on_hover_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=hover_node,
            on_exit=[circle_node, message_node, data_node, delayed_recording]
        )
    )

    return LaunchDescription([
        hover_node,
        on_hover_exit
    ])


# def generate_launch_description():
#     hover_node = Node(
#         package='trajectory_driver',
#         executable='hover',
#         name='hover',
#         output='screen'
#     )
#     terminate_hover = ExecuteProcess(
#         cmd=['pkill', '-f', 'hover'],
#         output='screen'
#     )
#     rosbag = ExecuteProcess(
#             cmd=[
#                 'ros2', 'bag', 'record',
#                 '-o', 'cir_traj_r0.2_w2_c00_h0.4_fanhigh',
#                 '/drone/combined_data'
#             ],
#             output='screen'
#         )
#     delayed_recording = TimerAction(
#         period = 3.0, 
#         actions=[rosbag]
#     )

#     main_nodes = [
        
#         Node(
#             package='trajectory_driver',  
#             executable='circle_simple',  # This should match the name in setup.py
#             name='driveCircle', # name of the node
#             output='screen',
#             parameters = [{'radius':0.2,'height':-0.4}]
#         ),
#         Node(
#             package='trajectory_driver', 
#             executable='message_node',  
#             name='message_node',
#             output='screen',
#             # parameters=[
#             #     {'radius': 0.2},
#             #     {'height': -0.4},
#             #     {'center_x': 0.0},
#             #     {'center_y': 0.0},
#             #     {'angular_vel': 1.0},
#             # ]
#         ),
#         Node(
#             package='trajectory_driver',  
#             executable='data_node',  
#             name='data',
#             output='screen',
#             # parameters=[
#             #     {'radius': 0.2},
#             #     {'height': -0.4},
#             #     {'center_x': 0.0},
#             #     {'center_y': 0.0},
#             #     {'angular_vel': 1.0},
#             #     {'g': 9.81},
#             #     {'kv': 7.4},
#             #     {'kx': 14},
#             #     {'m': 0.681},
#             # ]
#         ),
        
#         delayed_recording
#     ]

#     return LaunchDescription([
#         hover_node,
#         TimerAction(
#             period=5.0,
#             actions=[
#                 terminate_hover,
#                 LogInfo(msg="Hover node terminated, starting other nodes."),
#                 *main_nodes
#             ]
#         )
#     ])
