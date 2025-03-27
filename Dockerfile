# Use the official ROS 2 Humble base image for ARM64
FROM ros:humble-ros-base

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/root/ros2_ws

# Install dependencies
RUN apt update && apt install -y \
    # ROS tools
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-msgs \
    ros-humble-rclcpp-action \
    ros-humble-rclcpp-lifecycle \
    # Build tools for C++ and Python
    build-essential \
    cmake \
    gcc \
    g++ \
    make \
    # Install Python 3.10 and pip
    python3.10 \
    python3.10-dev \
    python3.10-venv \
    python3-pip \
    python3-colcon-common-extensions \
    usbutils \
    udev \
    # Install Redis
    redis-server \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3.10 as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 && \
    update-alternatives --set python3 /usr/bin/python3.10

# Install Python packages
RUN pip install --no-cache-dir \
    pyserial \
    numpy \
    opencv-python-headless \
    websockets \
    redis  # Install Redis Python client

# Set up ROS 2 dependencies
RUN rosdep update && rosdep fix-permissions

# Fix: Properly source ROS before running ros2 run
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

# Create ROS 2 workspace
RUN mkdir -p $ROS_WS/src && cd $ROS_WS && colcon build

# Source workspace automatically in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

# Set Redis to start automatically in the background
RUN echo "redis-server --daemonize yes" >> /root/.bashrc

# Set working directory
WORKDIR $ROS_WS

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
