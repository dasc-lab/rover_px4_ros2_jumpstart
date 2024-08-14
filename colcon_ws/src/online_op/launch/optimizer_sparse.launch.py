from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():
    optimizer_sparse = Node(
        
        package='online_op',
        executable='optimizer_sparse_gp',
        name='optimizer_sparse_gp',
        # namespace='',
        output='screen'
        # parameters=[{'param_name': 'param_value'}],
            
       
    )
    return LaunchDescription([
        optimizer_sparse
    ])
