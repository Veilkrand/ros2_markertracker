from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable

from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def generate_launch_description():
    ns = "/ros2_markertracker"
    # package_prefix = get_package_prefix('ros2_markertracker')
    # package_prefix = get_package_share_directory('ros2_markertracker')

    return LaunchDescription([
        Node(
            package='ros2_markertracker',
            executable='markertracker_node',
            output="screen",
            namespace=ns,
            parameters=[{

                "input_image_topic": "/camera/image_raw",
                "publish_topic_image_result": "image_result",
                "path_to_camera_file": '/home/ubuntu/ros2_ws/src/ros2_markertracker/ros2_markertracker/calibration/camerav2_1280x720.yaml',
                "marker_length": 0.1,

                # TODO: refactor to blacklist/whitelist array
                "ignore_marker_ids_array": 17,
            }],
        ),

    ])

