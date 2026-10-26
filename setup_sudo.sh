#!/usr/bin/env bash
# Run once inside Ubuntu-24.04 WSL (needs sudo). Installs build deps, Gazebo Harmonic, joy, stress-ng.
set -e
sudo apt-get update
sudo apt-get install -y git cmake ninja-build python3-pip python3-venv ccache astyle \
  gstreamer1.0-plugins-bad gstreamer1.0-libav libfuse2 stress-ng \
  ros-jazzy-joy ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
sudo wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic
echo "sudo part done"
