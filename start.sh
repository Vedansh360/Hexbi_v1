#!/bin/bash

# Start WebSocket server in background
echo "Starting WebSocket server..."
python3 websocket_server.py &

echo "Starting Camera server..."
python3 websocket_client_camera.py &

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
