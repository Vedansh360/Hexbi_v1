#!/bin/bash

echo "Stopping WebSocket server and ROS 2 nodes..."

# Kill WebSocket server
pkill -f ws_server_motor.py
pkill -f ws_server_camera.py
pkill -f ws_server_imu.py

# Kill all running ROS 2 nodes
pkill -f ros2

# Kill any lingering Python processes (optional, be careful)
pkill -f python3

echo "All processes stopped."

clear
