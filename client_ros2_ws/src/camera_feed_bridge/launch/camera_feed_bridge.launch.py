from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_feed_bridge',
            executable='camera_feed_bridge',
            name='camera_feed_publisher',
            output='screen'
        )
    ])
