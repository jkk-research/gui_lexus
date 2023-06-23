from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_lexus',
            executable='translator_joy_vel',
            output='screen',
            parameters=[
                {"input_device": "joystick1"},
                {"steer_gain": 6.0},
                {"accel_gain": 1.0},
                {"brake_gain": 1.0},
            ],
        )
    ])
