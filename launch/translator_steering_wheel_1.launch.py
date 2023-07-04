from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_lexus',
            executable='translator_joy_vel',
            output='screen',
            parameters=[
                {"input_device": "steering_wheel1"},
                {"steer_gain": 5.2},
                {"accel_gain": 1.0},
                {"brake_gain": 1.0},
            ],
        )
    ])
