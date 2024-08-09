from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():
    optimizer = Node(
        
        package='online_op',
        executable='optimizer',
        name='optimizer',
        # namespace='',
        output='screen'
        # parameters=[{'param_name': 'param_value'}],
            
       
    )
    return LaunchDescription([
        optimizer
    ])
