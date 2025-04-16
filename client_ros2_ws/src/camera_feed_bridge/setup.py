from setuptools import setup

package_name = 'camera_feed_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/redis_image_publisher.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vedansh Mishra',
    maintainer_email='vedanshmishra90@gmail.com',
    description='Bridge between Redis pub-sub camera feed and ROS 2 image topic.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_feed_bridge = camera_feed_bridge.camera_feed_bridge:main',
        ],
    },
)
