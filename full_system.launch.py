from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='BREE497',
            executable='serial_bridge',
            name='serial_bridge_node',
            output='screen'
        ),
        Node(
            package='BREE497',
            executable='odometry',
            name='odometry_node',
            output='screen'
        ),
        Node(
            package='BREE497',
            executable='motion_controller',
            name='motion_controller_node',
            output='screen'
        ),
        Node(
            package='BREE497',
            executable='mecanum_kinematics',
            name='mecanum_kinematics_node',
            output='screen'
        ),
        Node(
            package='BREE497',
            executable='safety',
            name='safety_node',
            output='screen'
        ),
        Node(
            package='BREE497',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen'
        ),
    ])
