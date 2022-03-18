from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_driver',
            #namespace='',
            executable='imu_driver',
            name='imu_driver',
            #prefix=['sudo -E']
        ),
        Node(
            package='motor_driver',
            #namespace='',
            executable='motor_driver',
            name='motor_driver'
        ),
        Node(
            package='onnx_controller',
            executable='onnx_controller',
            name='onnx_controller',
            # remappings=[
            #     ('/input/pose', '/turtlesim1/turtle1/pose'),
            #     ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            # ]
        )
    ])