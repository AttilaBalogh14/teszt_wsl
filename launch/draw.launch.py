from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='bal_hrf_kisbeadando',  # cseréld ki a megfelelő csomagnevére
            executable='diamond_draw',       # vagy a megfelelő node-ra
            output='screen',
        ),
    ])
