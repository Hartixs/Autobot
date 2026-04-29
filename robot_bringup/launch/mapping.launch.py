# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
    
#     bringup_dir = get_package_share_directory('robot_bringup')
#     slam_toolbox_dir = get_package_share_directory('slam_toolbox')

#     # --- NEW: Path to your custom SLAM configuration ---
#     slam_params_file = os.path.join(bringup_dir, 'config', 'mapper_params.yaml')

#     return LaunchDescription([
        
#         # 1. Start your physical robot, sensors, and EKF
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(bringup_dir, 'launch', 'ultimate_launch.launch.py')
#             )
#         ),
        
#         # 2. Start SLAM Toolbox to map the environment
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
#             ),
#             launch_arguments={
#                 'use_sim_time': 'false',
#                 'slam_params_file': slam_params_file  # <--- WE ADDED THIS LINE
#             }.items()
#         )
#     ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                # Override the default frames to match your URDF and ESKF
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'base_frame': 'base_footprint'},
                {'scan_topic': '/scan'},
                {'mode': 'mapping'},
                
                # Tuning parameters for a cleaner map
                {'use_scan_matching': True},
                {'resolution': 0.05}, # 5cm per map pixel
                {'minimum_travel_distance': 0.1}, # Update map every 10cm of movement
                {'minimum_travel_heading': 0.174} # Update map every ~10 degrees of turn
            ]
        )
    ])