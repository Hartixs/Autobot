## **Installation & Build Instructions**
Clone or copy all the files provided in this repository into the src folder of our main ROS 2 workspace. (Except the website folder)

Navigate to the root of the workspace and build the package using colcon:

Bash

_cd ~/our_workspace
colcon build_
Source the workspace overlay to make the packages visible to ROS 2:

Bash

_source install/setup.bash_

## **Running the System**

### 1. Bringup the Base Robot
To launch the hardware bridges, ESKF, LiDAR, and core transformations, we use our main launch file:

Bash

_ros2 launch <our_package_name> ultimate.launch.py_
(Note: Replace <our_package_name> with the actual name of our CMake package).

### 2. Mapping (SLAM)
To map a new environment, we run the asynchronous online mapping node provided by slam_toolbox. In a new terminal, source the workspace and run:

Bash

_ros2 launch slam_toolbox online_async_launch.py_
Once the map is completely drawn in RViz, remember to save it using the slam_toolbox panel or via the map saver CLI before shutting down the node.

### 3. Autonomous Navigation
Once we have a saved map of the room, we can launch the Nav2 stack for autonomous path planning and dynamic obstacle avoidance. Run the following command, pointing it to our saved map and custom parameters file:

Bash

_ros2 launch nav2_bringup bringup_launch.py map:=/path/to/our/map.yaml params_file:=/path/to/our/nav2_params.yaml_
