from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    optimizer = Node(
        
        package='onlin_op',
        executable='optimizer',
        name='optimizer',
        # namespace='',
        output='screen'
        # parameters=[{'param_name': 'param_value'}],
            
       
    )
    return LaunchDescription([
        optimizer
    ])
