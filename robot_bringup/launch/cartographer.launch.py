import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Define directories
    bringup_dir = get_package_share_directory('robot_bringup')
    
    # Path to your config folder where slam.lua is stored
    cartographer_config_dir = os.path.join(bringup_dir, 'config')
    cartographer_config_basename = 'slam.lua'

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('resolution', default_value='0.05', description='Resolution of a grid cell'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0', description='OccupancyGrid publishing period'),

        # 1. Start your physical robot, sensors, and ESKF 
        # (This is identical to what mapping.launch.py was doing)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'ultimate_launch.launch.py')
            )
        ),

        # 2. Start the Core Cartographer SLAM Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_basename
            ]
        ),

        # 3. Start Cartographer Occupancy Grid Node
        # (This translates Cartographer's internal submaps into a standard ROS 2 /map topic)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec
            ]
        )
    ])