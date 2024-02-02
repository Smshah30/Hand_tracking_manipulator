from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator',
            executable='fkine',
            name='fkine'
        ),
        Node(
            package='manipulator',
            executable='invkine',
            name='invkine'
        )
        # Node(
            # package='manipulator',
            # executable='connector_arm_hand',
            # name='connector_arm_hand'
        # )
    ])