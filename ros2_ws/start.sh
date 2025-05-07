#!/bin/bash

# Start WebSocket server in background
echo "Starting Motor Control server..."
python3 ws_server_motor.py &

echo "Starting Camera server..."
python3 ws_server_camera.py &

# Start ROS 2 node: imu_publisher in background
echo "Starting imu_publisher node..."
ros2 run imu_publisher imu_publisher &

echo "Starting IMU server..."
python3 ws_server_imu.py &

# Start ROS 2 node: keyboard_input in background
echo "Starting keyboard_input node..."
ros2 run motor_control keyboard_input &

# Start ROS 2 node: motor_controller in background
echo "Starting motor_controller node..."
ros2 run motor_control motor_controller &

# Print all running background processes
echo "All processes started in background."
jobs

# Prevent script from exiting immediately
wait
