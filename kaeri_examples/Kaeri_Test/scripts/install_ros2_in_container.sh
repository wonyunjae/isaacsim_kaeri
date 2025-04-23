#!/bin/bash
set -e

echo "=== Installing ROS2 Humble in Isaac Sim Container ==="

# Add ROS2 apt repository
apt update && apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble base
apt update
apt install -y ros-humble-ros-base python3-colcon-common-extensions

# Install ROS2 Python client library
apt install -y python3-rclpy

# Source ROS2 setup in bashrc
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "=== ROS2 Humble installation completed ==="
echo "To use ROS2, please restart your terminal or run: source /opt/ros/humble/setup.bash"
