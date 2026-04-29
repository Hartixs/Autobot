import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    # 1. Start your IMU Node
        Node(
        package='imu_setup_py',
        executable='mpu6050_visualizer', # Make sure this matches your setup.py entry point
        name='mpu6050_node',
        output='screen',
        ),

        # 2. Start the Filter (Madgwick) to create the orientation
        # This takes raw data and makes it "rotate"
        Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{'use_mag': False, 'publish_tf': True, 'world_frame': 'world'}],
        remappings=[
        ('/imu/data_raw', '/imu/data'), # Map your node output to filter input
        ]
        ),

        # 3. Static Transform (Base to IMU)
        # This tells ROS where the IMU is sitting on the "robot"
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
        ),

        # 4. Open RViz2 with a blank config
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        )
        ])