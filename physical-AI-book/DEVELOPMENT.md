# Development Environment Setup

This guide outlines the setup process for the Physical AI & Humanoid Robotics book development environment.

## Prerequisites

- **Node.js** (version 18 or higher)
- **Python 3.11** (for ROS 2 Humble)
- **ROS 2 Humble Hawksbill** (for robotics examples)
- **Gazebo Garden** (for simulation examples)
- **Isaac Sim** (for advanced perception examples)
- **Docker** (for consistent environments)

## Node.js Setup

The frontend is built with Docusaurus, which requires Node.js:

```bash
# Install Node.js (version 18 or higher)
# Using nvm (recommended):
nvm install 18
nvm use 18

# Or install directly from nodejs.org
```

## ROS 2 Humble Setup

For robotics examples and code validation:

```bash
# On Ubuntu 22.04
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/rosSigning.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
```

## Gazebo Garden Setup

For simulation examples:

```bash
# Install Gazebo Garden
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

## Isaac Sim Setup

For advanced perception examples (optional):

```bash
# Isaac Sim requires NVIDIA GPU and specific drivers
# Follow the official Isaac Sim installation guide:
# https://docs.omniverse.nvidia.com/isaacsim/latest/installation_guide/index.html
```

## Project Setup

1. Clone the repository:
```bash
git clone <repository-url>
cd physical-AI-book
```

2. Install Node.js dependencies:
```bash
cd physical-AI-book
npm install
```

3. Start the development server:
```bash
npm start
```

## Docker Environment (Optional but Recommended)

For consistent development and testing across different environments:

```dockerfile
# Dockerfile for consistent development environment
FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]

# Install Node.js
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash -
RUN apt-get install -y nodejs

# Install Python packages
RUN pip3 install --upgrade pip
RUN pip3 install pytest

# Set up workspace
WORKDIR /workspace
COPY . .
RUN cd physical-AI-book && npm install

EXPOSE 3000
CMD ["bash"]
```

## Verification

To verify your setup:

1. Start the Docusaurus server: `npm start`
2. Navigate to `http://localhost:3000`
3. Verify you can see the Physical AI book interface
4. Check that all examples and components load correctly