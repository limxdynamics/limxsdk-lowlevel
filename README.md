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

## 4. C++ Quick Start

The `examples/` directory contains ready-to-build C++ examples with their own `CMakeLists.txt`.

### Build and Run

```bash
cd limxsdk-lowlevel
mkdir build && cd build
cmake ..
make

# Control a single joint
./examples/pf_joint_move <robot_ip>

# Control all joints simultaneously
./examples/pf_groupJoints_move <robot_ip>
```

For simulation, use `127.0.0.1` as the robot IP. Before running, set your robot type, e.g. `export ROBOT_TYPE=SF_TRON1A` (see available models in [tron1-robot-description](https://github.com/limxdynamics/tron1-robot-description)).

### How the Examples Work

Each example inherits from `PFControllerBase` and overrides `init()` and `starting()`:

```cpp
#include "pf_controller_base.h"

class PFJointMove : public PFControllerBase {
public:
    void init() {
        // Wait for calibration before commanding joints
        pf_->subscribeDiagnosticValue([&](const limxsdk::DiagnosticValueConstPtr& msg) {
            if (msg->name == "calibration" && msg->code != 0) abort();
        });
    }

    void starting() {
        while (true) {
            if (robotstate_on_) {
                // Read current joint position from robot state
                double currentPos = robot_state_.q[joint_id];
                // Send position command to a single joint
                singleJointController(joint_id, kp, kd, targetPos, targetVel, targetTorque);
            }
        }
    }
};
```

Key methods provided by `PFControllerBase`:
- `singleJointController(id, kp, kd, pos, vel, torque)` — control one joint
- `groupJointController(kp, kd, pos, vel, torque)` — control all joints
- `robot_state_.q[id]` — read current joint position
- `robotstate_on_` — true when robot state data is available
- `pf_->subscribeDiagnosticValue(cb)` — subscribe to diagnostic events

## 5. API Overview

The SDK provides robot-specific classes (all singletons) under the `limxsdk` namespace:

| Class | Header | Robot Type |
|-------|--------|------------|
| `limxsdk::PointFoot` | `pointfoot.h` | TRON1 biped / wheel-foot |
| `limxsdk::Humanoid` | `humanoid.h` | Humanoid (Oli) |
| `limxsdk::Wheellegged` | `wheellegged.h` | Wheel-legged platforms |
| `limxsdk::Tron2` | `tron2.h` | TRON2 |

Each robot class inherits from `ApiBase` and provides:

```cpp
auto* robot = limxsdk::PointFoot::getInstance();
robot->init("127.0.0.1");                 // Connect to robot (use 127.0.0.1 for sim)

// Subscription-based reads (callback pattern)
robot->subscribeImuData([](auto& imu) { /* handle IMU */ });
robot->subscribeRobotState([](auto& state) { /* state.q[i] = joint position */ });
robot->subscribeDiagnosticValue([](auto& msg) { /* calibration, errors, etc. */ });

// Command-based writes
robot->publishRobotCmd(cmd);              // Send joint commands to the robot
robot->setRobotLightEffect(effect);       // Control robot LEDs

// Utility
int n = robot->getMotorNumber();          // Number of motors
auto names = robot->getMotorNames();      // Motor/joint names
```

