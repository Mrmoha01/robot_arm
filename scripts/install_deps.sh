#!/usr/bin/env bash
# Install the ROS 2 Jazzy / Gazebo Harmonic packages this project needs (Ubuntu 24.04).
set -euo pipefail

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy not found in /opt/ros/jazzy. Install ros-jazzy-desktop first:"
  echo "  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
  exit 1
fi

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pytest \
  python3-numpy \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2

echo "Done. Next: source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
