# English | [中文](README_cn.md)
# LimX SDK Usage Guide

## 1. Set Up Development Environment

For algorithm developers, we recommend setting up a ROS Noetic-based development environment on **Ubuntu 20.04**. ROS provides a suite of tools and libraries—such as core libraries, communication frameworks, and simulation tools like **Gazebo**—which greatly simplify the development, testing, and deployment of robot algorithms. These resources offer a rich and comprehensive environment for algorithm development.

Of course, even without ROS, you can still develop your motion control algorithm in other environments. The motion control SDK we provide is **dependency-free**, based on standard **C++11 and Python**, and supports cross-platform and cross-OS development, offering developers greater flexibility.

To install ROS Noetic, please refer to the official documentation:  
👉 [https://wiki.ros.org/noetic/Installation/Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)  
Select `ros-noetic-desktop-full` for installation.

Once ROS Noetic is installed, run the following shell command in a terminal to install the required dependencies:

```
sudo apt-get update
sudo apt install ros-noetic-urdf \
                 ros-noetic-kdl-parser \
                 ros-noetic-urdf-parser-plugin \
                 ros-noetic-hardware-interface \
                 ros-noetic-controller-manager \
                 ros-noetic-controller-interface \
                 ros-noetic-controller-manager-msgs \
                 ros-noetic-control-msgs \
                 ros-noetic-ros-control \
                 ros-noetic-gazebo-* \
                 ros-noetic-rqt-gui \
                 ros-noetic-rqt-controller-manager \
                 ros-noetic-plotjuggler* \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                 python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
```

## 2. Create Workspace

Follow the steps below to create an algorithm development workspace:

### Step 1: Open a terminal

### Step 2: Create a new workspace directory

```bash
mkdir -p ~/limx_ws/src
```

### Step 3: Clone required repositories

#### Clone the motion control SDK:

```bash
cd ~/limx_ws/src
git clone https://github.com/limxdynamics/limxsdk-lowlevel.git
```

#### Clone Gazebo simulator plugins:

```bash
cd ~/limx_ws/src
git clone https://github.com/limxdynamics/pointfoot-gazebo-ros.git
```

#### Clone robot model description files:

```bash
cd ~/limx_ws/src
git clone https://github.com/limxdynamics/robot-description.git
```

#### Clone visualization debugging tools:

```bash
cd ~/limx_ws/src
git clone https://github.com/limxdynamics/robot-visualization.git
```

### Step 4: Compile the workspace

```bash
cd ~/limx_ws
catkin_make install
```

## 3. Python Motion Control SDK

### 3.1 Overview

We provide a Python interface with the same functionality as the C++ SDK. This allows developers unfamiliar with C++ to write motion control algorithms in Python. Python’s simplicity, clear syntax, and rich third-party ecosystem enable developers to get started quickly and iterate faster.

With the Python interface, developers can benefit from:

- Rapid prototyping and testing
- Cross-platform support
- Easy integration of reinforcement learning (RL) models into both simulation and real hardware environments

This flexibility accelerates algorithm development and deployment.

### 3.2 Install Python SDK

Please install the appropriate `.whl` file depending on your platform:

#### On Linux x86_64:

```bash
pip install python3/amd64/limxsdk-*-py3-none-any.whl
```

#### On Linux aarch64:

```bash
pip install python3/aarch64/limxsdk-*-py3-none-any.whl
```

#### On Windows:

```bash
pip install python3/win/limxsdk-*-py3-none-any.whl
```

### 3.3 Python Example

You can refer to the example Python script here:  
👉 [Example Code on GitHub](https://github.com/limxdynamics/limxsdk-lowlevel/blob/master/python3/amd64/example.py)
