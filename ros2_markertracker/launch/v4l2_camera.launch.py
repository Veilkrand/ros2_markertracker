

from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    ns = "/camera" 
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output="screen",
            namespace=ns,
            parameters=[{"image_size": [1280, 720], "rotate": 0, "vertical_flip": True, "horizontal_flip": True}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "camera"],
            output="screen")

    ])

