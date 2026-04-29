# # import os
# # from ament_index_python.packages import get_package_share_directory
# # from launch import LaunchDescription
# # from launch.actions import IncludeLaunchDescription
# # from launch.launch_description_sources import PythonLaunchDescriptionSource

# # def generate_launch_description():
    
# #     # Paths to the packages
# #     bringup_dir = get_package_share_directory('robot_bringup')
# #     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
# #     # EXACT ABSOLUTE PATH TO YOUR SAVED MAP
# #     map_file_path = '/home/rbcnvamsi/Desktop/pbl/map_name.yaml' 
    
# #     # PATH TO YOUR NEW CUSTOM NAV2 PARAMETERS
# #     nav2_params_path = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

# #     return LaunchDescription([
        
# #         # 1. Start your physical robot
# #         IncludeLaunchDescription(
# #             PythonLaunchDescriptionSource(ros2
# #                 os.path.join(bringup_dir, 'launch', 'ultimate_launch.launch.py')
# #             )
# #         ),
        
# #         # 2. Start the Nav2 Stack with your map AND your custom parameters
# #         IncludeLaunchDescription(
# #             PythonLaunchDescriptionSource(
# #                 os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
# #             ),
# #             launch_arguments={
# #                 'map': map_file_path,
# #                 'use_sim_time': 'false',
# #                 'params_file': nav2_params_path  # <--- THIS IS THE NEW LINE
# #             }.items()
# #         ),
# #         Node(
# #     package='robot_localization',
# #     executable='ekf_node',
# #     name='ekf_filter_node',
# #     output='screen',
# #     parameters=[ekf_config_path],
# #     # Remap the output so standard Nav2 tools see the fused odometry
# #     remappings=[('odometry/filtered', 'odom')]
# # )
# #     ])


# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
    
#     # Paths to the packages
#     bringup_dir = get_package_share_directory('robot_bringup')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
#     # EXACT ABSOLUTE PATH TO YOUR SAVED MAP
#     map_file_path = '/home/rbcnvamsi/Desktop/pbl/maps.yaml' 
    
#     # PATH TO YOUR NEW CUSTOM NAV2 PARAMETERS
#     nav2_params_path = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

#     return LaunchDescription([
        
#         # 1. Start your physical robot, sensors, and EKF
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(bringup_dir, 'launch', 'ultimate_launch.launch.py')
#             )
#         ),
        
#         # 2. Start the Nav2 Stack with your map AND your custom parameters
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
#             ),
#             launch_arguments={
#                 'map': map_file_path,
#                 'use_sim_time': 'false',
#                 'params_file': nav2_params_path
#             }.items()
#         )
#     ])
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# def generate_launch_description():

#     # ── Package paths ──────────────────────────────────────────────────────────
#     bringup_dir     = get_package_share_directory('robot_bringup')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')

#     # ── Map & Nav2 params ──────────────────────────────────────────────────────
#     # Change this to the full absolute path of your saved map YAML file.
#     map_file_path  = '/home/rbcnvamsi/Desktop/pbl/map_name.yaml'
#     nav2_params_path = os.path.join(bringup_dir, 'config', 'navparams.yaml')

#     return LaunchDescription([

#         # ── 1. Full robot bringup ──────────────────────────────────────────────
#         # Starts: robot_state_publisher, base_controller, imu_node,
#         #         ekf_filter_node, sllidar_node, rviz2.
#         # The EKF node lives here — not in navigate_launch — so that filtered
#         # odometry is always available regardless of whether Nav2 is running.
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(bringup_dir, 'launch', 'ultimate_launch.launch.py')
#             )
#         ),

#         # ── 2. Nav2 stack ─────────────────────────────────────────────────────
#         # bringup_launch.py starts AMCL, costmaps, planners, controllers, etc.
#         # use_sim_time must be false on real hardware.
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
#             ),
#             launch_arguments={
#                 'map':          map_file_path,
#                 'use_sim_time': 'false',
#                 'params_file':  nav2_params_path,
#             }.items(),
#         ),
#     ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # ─── Directories ──────────────────────────────────────────────────────────
    bringup_dir = get_package_share_directory('robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ─── Setup Launch Configurations (Dynamic Arguments) ──────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_dir = LaunchConfiguration('map')
    param_dir = LaunchConfiguration('params_file')

    # Define your default paths
    default_map_path = '/home/rbcnvamsi/Desktop/pbl/map_name.yaml'
    default_param_path = os.path.join(bringup_dir, 'config', 'official_nav_params.yaml')

    return LaunchDescription([
        
        # ─── Declare Arguments (Allows overriding via terminal) ───────────────
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false', 
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map', 
            default_value=default_map_path, 
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'params_file', 
            default_value=default_param_path, 
            description='Full path to the ROS2 parameters file to use'
        ),

        # ─── 1. Full Robot Bringup ────────────────────────────────────────────
        # Starts: robot_state_publisher, base_controller, imu_node,
        # eskf_localization_node, sllidar_node, rviz2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'ultimate_launch.launch.py')
            )
        ),

        # ─── 2. Nav2 Stack Bringup ────────────────────────────────────────────
        # Starts AMCL, costmaps, planners, controllers, etc.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        )
    ])