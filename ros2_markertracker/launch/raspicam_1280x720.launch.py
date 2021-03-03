from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    ns = "/ros2_markertracker"
    return LaunchDescription([
        Node(
            package='ros2_markertracker',
            executable='markertracker_node',
            output="screen",
            namespace=ns,
            parameters=[{

                "input_image_topic": "camera/image_raw",
                "publish_topic_image_result": "$(find markertracker_node)/calibration/raspicam_v2_640x480.yaml",
                "marker_length": 10,
                # TODO: refactor to blacklist/whitelist array
                "ignore_marker_ids_array": 17,
            }],
        ),

    ])

