#!/bin/bash

echo "Stopping WebSocket server and ROS 2 nodes..."

# Kill WebSocket server
pkill -f websocket_server.py

# Kill all running ROS 2 nodes
pkill -f ros2

# Kill any lingering Python processes (optional, be careful)
pkill -f python3

echo "All processes stopped."
