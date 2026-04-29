# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# import xacro

# def generate_launch_description():
#     # 1. Parse the URDF/Xacro file
#     pkg_description = get_package_share_directory('robot_description')
#     pkg_bringup = get_package_share_directory('robot_bringup')
#     # Make sure this points to your MAIN xacro file
#     xacro_file = os.path.join(pkg_description, 'urdf', 'tortoisebot_simple.xacro') 
    
#     robot_description_config = xacro.process_file(xacro_file)
#     robot_description_xml = robot_description_config.toxml()

#     rviz_config_path = os.path.join(pkg_bringup, 'rviz', 'urdf_config.rviz')

#     return LaunchDescription([
#         # Node 1: Robot State Publisher (Broadcasts the URDF and static TF tree)
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             parameters=[{'robot_description': robot_description_xml}]
#         ),
        
#         # Node 2: Your Base Controller (Handles Motors, Encoders, and Odometry TF)
#         Node(
#             package='robot_base',
#             executable='base_controller', # Use the executable name you set in setup.py
#             name='base_controller'
#         ),
        
#         # Node 3: Your IMU Visualizer
#         Node(
#             package='imu_setup_py', # Assuming this is your package name
#             executable='mpu6050_visualizer', # Assuming this is your executable name
#             name='imu_node'
#         ),
        
#         # Node 4: RPLidar
#         Node(
#             package='sllidar_ros2',
#             executable='sllidar_node',
#             name='sllidar_node',
#             parameters=[{
#                 'serial_port': '/dev/ttyUSB1',
#                 'serial_baudrate': 115200, # Assuming Lidar is USB1, Arduino is USB0
#                 'frame_id': 'lidar',           # Matches the link name in your URDF snippet
#                 'angle_compensate': True,
#                 'scan_mode': 'Standard'
#             }]
#         ),


#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_path]
#         )
#     ])


# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# import xacro

# def generate_launch_description():
#     # 1. Parse the URDF/Xacro file
#     pkg_description = get_package_share_directory('robot_description')
#     pkg_bringup = get_package_share_directory('robot_bringup')
    
#     # Paths
#     xacro_file = os.path.join(pkg_description, 'urdf', 'tortoisebot_simple.xacro') 
#     rviz_config_path = os.path.join(pkg_bringup, 'rviz', 'urdf_config.rviz')
#     ekf_config_path = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    
#     # Process Xacro
#     robot_description_config = xacro.process_file(xacro_file)
#     robot_description_xml = robot_description_config.toxml()

#     return LaunchDescription([
#         # Node 1: Robot State Publisher (Broadcasts the URDF)
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             parameters=[{'robot_description': robot_description_xml}]
#         ),
        
#         # Node 2: Joint State Publisher (CRITICAL: Fixes floating TF islands)
#         Node(
#             package='joint_state_publisher',
#             executable='joint_state_publisher',
#             name='joint_state_publisher'
#         ),
        
#         # Node 3: Your Base Controller (Handles Motors & Encoders -> /odom_unfiltered)
#         Node(
#             package='robot_base',
#             executable='base_controller', 
#             name='base_controller'
#         ),
        
#         # Node 4: Your IMU Visualizer (-> /imu/data)
#         Node(
#             package='imu_setup_py', 
#             executable='mpu6050_visualizer', 
#             name='imu_node'
#         ),
        
#         # Node 5: RPLidar
#         Node(
#             package='sllidar_ros2',
#             executable='sllidar_node',
#             name='sllidar_node',
#             parameters=[{
#                 'serial_port': '/dev/ttyUSB1',
#                 'serial_baudrate': 115200, 
#                 'frame_id': 'lidar',           
#                 'angle_compensate': True,
#                 'scan_mode': 'Standard'
#             }]
#         ),

#         # Node 6: The EKF (Fuses Odometry + IMU, broadcasts odom -> base_link)
#         Node(
#             package='robot_localization',
#             executable='ekf_node',
#             name='ekf_filter_node',
#             output='screen',
#             parameters=[ekf_config_path],
#             remappings=[('odometry/filtered', 'odom')]
#         ),

#         # Node 7: RViz2
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_path]
#         )
#     ])

# import os
# import xacro
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():

#     # ── Package paths ──────────────────────────────────────────────────────────
#     pkg_description = get_package_share_directory('robot_description')
#     pkg_bringup     = get_package_share_directory('robot_bringup')

#     # ── URDF ──────────────────────────────────────────────────────────────────
#     xacro_file = os.path.join(pkg_description, 'urdf', 'tortoisebot_simple.xacro')
#     robot_description_xml = xacro.process_file(xacro_file).toxml()

#     # ── Config / RViz paths ────────────────────────────────────────────────────
#     rviz_config_path = os.path.join(pkg_bringup, 'rviz', 'urdf_config.rviz')
    
#     return LaunchDescription([

#         # ── 1. Robot State Publisher ───────────────────────────────────────────
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'robot_description': robot_description_xml}],
#         ),

#         # ── 2. Base Controller ─────────────────────────────────────────────────
#         Node(
#             package='robot_base',
#             executable='base_controller',
#             name='base_controller',
#             output='screen',
#             remappings=[
#                 ('/odom', '/odom_unfiltered'),   
#             ],
#         ),

#         # ── 3. IMU Node (MPU6050) ──────────────────────────────────────────────
#         Node(
#             package='imu_setup_py',
#             executable='mpu6050_visualizer',
#             name='imu_node',
#             output='screen',
#         ),

#         # ── 4. Error State Kalman Filter (ESKF) ────────────────────────────────
#         # Replaces the old robot_localization EKF.
#         Node(
#             package='imu_setup_py',  # Assuming you move eskf.py here (see note below)
#             executable='eskf_node',  # Matches the entry point in setup.py
#             name='eskf_localization_node',
#             output='screen',
#         ),

#         # ── 5. RPLidar ────────────────────────────────────────────────────────
#         Node(
#             package='sllidar_ros2',
#             executable='sllidar_node',
#             name='sllidar_node',
#             output='screen',
#             parameters=[{
#                 'serial_port':      '/dev/lidar',
#                 'serial_baudrate':  115200,
#                 'frame_id':         'lidar',   
#                 'angle_compensate': True,
#                 'scan_mode':        'Standard',
#             }],
#         ),

#         # ── 6. RViz ───────────────────────────────────────────────────────────
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_path],
#             output='screen',
#         ),
#     ])

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # ── Package paths ──────────────────────────────────────────────────────────
    pkg_description = get_package_share_directory('robot_description')
    pkg_bringup     = get_package_share_directory('robot_bringup')

    # ── URDF ──────────────────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_description, 'urdf', 'tortoisebot_simple.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # ── Config / RViz paths ────────────────────────────────────────────────────
    rviz_config_path = os.path.join(pkg_bringup, 'rviz', 'urdf_config.rviz')
   
    return LaunchDescription([

        # ── 1. Robot State Publisher ───────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_xml}],
        ),

        # ── 2. Base Controller (Motors) ────────────────────────────────────────
        Node(
            package='robot_base',
            executable='base_controller',
            name='base_controller',
            output='screen'
        ),


        # ── 5. Error State Kalman Filter (ESKF) ────────────────────────────────
        Node(
            package='imu_setup_py',
            executable='eskf_node',
            name='eskf_localization_node',
            output='screen',
        ),

                # ── 5 Imu publisher) ────────────────────────────────
        Node(
            package='imu_setup_py',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
        ),

        # ── 6. RPLidar ────────────────────────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port':      '/dev/lidar', # Ensure your udev rules are set
                'serial_baudrate':  115200,
                'frame_id':         'lidar',  
                'angle_compensate': True,
                'scan_mode':        'Standard',
            }],
        ),

        # ── 7. RViz ───────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),
    ])