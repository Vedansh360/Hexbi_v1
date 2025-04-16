from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_feed_bridge',
            executable='redis_image_publisher',
            name='redis_image_publisher',
            output='screen'
        )
    ])
