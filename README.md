[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# beginner_tutorials
ROS2 tutorial for gazebo simulation.

## Assumptions
```
OS: Ubuntu Linux Focal (20.04) 64-bit
ROS2 Distro: Humble Hawksbill
ROS2 Workspace name: ros2_ws
ROS2 Installation Directory: ros2_humble
```

## ROS 2 dependencies
```
ament_cmake
rclcpp
std_msgs
```

## Creating the workspace and building the package
```
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Vishaal-Kanna/ROS2_Gazebo_sim.git
cd ..
colcon build --packages-select gazebo_sim
```

## Run Publisher and subsrciber
```
Open a terminal from ros2_ws
. install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo `/share/turtlebot3_gazebo/models/
ros2 launch gazebo_sim walker_launch.py
```

## To play recorded rosbag
```
cd <your_path_to_repo>/Results/Rosbag
ros2 bag play all_topics
ros2 bag info all_topics
```




