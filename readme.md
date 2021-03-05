# ROS2 Marker Tracker

## Build
```
colcon build --packages-select ros2_markertracker_interfaces ros2_markertracker
source install/setup.bash
```

## Launch Camera
```
ros2 launch src/ros2_markertracker/ros2_markertracker/launch/v4l2_camera.launch.py
```

## Run
```
ros2 launch src/ros2_markertracker/ros2_markertracker/launch/markertracker_raspicam_1280x720.launch.py
```

