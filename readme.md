# ROS2 Marker Tracker

## Build
colcon build --packages-select ros2_markertracker_interfaces
colcon build --packages-select ros2_markertracker
source install/setup.bash

## Camera
ros2 launch raspi_video.launch

## Run
ros2 launch src/ros2_markertracker/ros2_markertracker/launch/raspicam_1280x720.launch.py