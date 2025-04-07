from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the sequence publisher
        Node(
            package='arduino_servo_control',
            executable='sequence_publisher',
            name='sequence_publisher',
            output='screen'
        ),
        
        # Launch the servo control node
        Node(
            package='arduino_servo_control',
            executable='servo_control',
            name='servo_node',
            output='screen'
        )
    ]) 