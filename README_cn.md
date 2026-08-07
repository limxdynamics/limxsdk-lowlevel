# 中文 | [English](README.md)
# LimX SDK 使用说明

## 1. 搭建开发环境

在算法开发者自己的电脑中，我们推荐在 Ubuntu 20.04 操作系统上建立基于 ROS Noetic 的算法开发环境。ROS 提供了一系列工具和库，如核心库、通信库和仿真工具（如 Gazebo），极大地便利了机器人算法的开发、测试和部署。这些资源为用户提供了一个丰富而完整的算法开发环境。

当然，即使没有 ROS，您也可以选择在其他环境中开发自己的运动控制算法。我们提供的运动控制开发接口，是一个基于标准 C++11 和 Python 的无依赖 SDK。它支持跨操作系统和平台调用开发，为开发者提供了更灵活的选择。

ROS Noetic 安装请参考文档：https://wiki.ros.org/noetic/Installation/Ubuntu ，选择“ros-noetic-desktop-full”进行安装。ROS Noetic 安装完成后，Bash 终端输入以下 Shell 命令，安装开发环境所依赖的库：

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

## 2. 创建工作空间

可以按照以下步骤，创建一个算法开发工作空间：

- 打开一个 Bash 终端。

- 创建一个新目录来存放工作空间。例如，可以在用户的主目录下创建一个名为“limx_ws”的目录：

  ```
  mkdir -p ~/limx_ws/src
  ```

- 下载运动控制开发接口：

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/limxsdk-lowlevel.git
  ```

- 下载 Gazebo 仿真器：

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/pointfoot-gazebo-ros.git
  ```

- 下载机器人模型描述文件

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/robot-description.git
  ```

- 下载可视化调试工具

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/robot-visualization.git
  ```

- 编译工程：

  ```
  cd ~/limx_ws
  catkin_make install
  ```


## 3. Python 运动控制开发接口

### 3.1 概述

提供与 C++相同功能的[Python 运动算法开发接口](https://github.com/limxdynamics/limxsdk-lowlevel/tree/master/python3)，使得不熟悉 C++编程语言的开发者能够使用 Python 进行运动控制算法的开发。Python 语言易于学习，具有简洁清晰的语法和丰富的第三方库，使开发者能够更快速地上手并迅速实现算法。通过 Python 接口，开发者可以利用 Python 的动态特性进行快速原型设计和实验验证，加速算法的迭代和优化过程。同时，Python 的跨平台性和强大的生态系统支持，使得运动算法能够更广泛地应用于不同平台和环境。此外，RL（强化学习）模型的快速部署到仿真和真机环境中也得益于 Python 的灵活性，开发者可以使用 Python 轻松地将 RL 模型集成到各种仿真平台和真实硬件中，实现快速迭代和验证算法的性能。

### 3.2 安装运动控制开发库

请根据操作系统选择相应的命令：

- Linux x86_64 平台

  ```Bash
  pip install python3/amd64/limxsdk-*-py3-none-any.whl
  ```

- Linux aarch64 平台

  ```Bash
  pip install python3/aarch64/limxsdk-*-py3-none-any.whl
  ```

- Windows  平台

  ```Bash
  pip install python3/win/limxsdk-*-py3-none-any.whl
  ```

### 3.3 参考例程

Python 接口参考例程: https://github.com/limxdynamics/limxsdk-lowlevel/blob/master/python3/amd64/example.py

## 4. C++ 快速入门

`examples/` 目录包含可直接编译的 C++ 示例，自带 `CMakeLists.txt`。

### 编译和运行

```bash
cd limxsdk-lowlevel
mkdir build && cd build
cmake ..
make

# 控制单个关节
./examples/pf_joint_move <robot_ip>

# 同时控制所有关节
./examples/pf_groupJoints_move <robot_ip>
```

仿真场景下使用 `127.0.0.1` 作为机器人 IP。运行前请先设置机器人型号，例如 `export ROBOT_TYPE=SF_TRON1A`（可用型号见 [tron1-robot-description](https://github.com/limxdynamics/tron1-robot-description)）。

### 示例代码原理

每个示例继承自 `PFControllerBase`，重写 `init()` 和 `starting()` 方法：

```cpp
#include "pf_controller_base.h"

class PFJointMove : public PFControllerBase {
public:
    void init() {
        // 等待标定完成后才能发送关节指令
        pf_->subscribeDiagnosticValue([&](const limxsdk::DiagnosticValueConstPtr& msg) {
            if (msg->name == "calibration" && msg->code != 0) abort();
        });
    }

    void starting() {
        while (true) {
            if (robotstate_on_) {
                // 从机器人状态读取当前关节位置
                double currentPos = robot_state_.q[joint_id];
                // 向单个关节发送位置指令
                singleJointController(joint_id, kp, kd, targetPos, targetVel, targetTorque);
            }
        }
    }
};
```

`PFControllerBase` 提供的关键方法：
- `singleJointController(id, kp, kd, pos, vel, torque)` — 控制单个关节
- `groupJointController(kp, kd, pos, vel, torque)` — 控制全部关节
- `robot_state_.q[id]` — 读取当前关节位置
- `robotstate_on_` — 机器人状态数据是否就绪
- `pf_->subscribeDiagnosticValue(cb)` — 订阅诊断事件

## 5. API 概览

SDK 在 `limxsdk` 命名空间下提供了针对不同机器人类型的控制类（均为单例模式）：

| 类 | 头文件 | 适用机器人 |
|-------|--------|------------|
| `limxsdk::PointFoot` | `pointfoot.h` | TRON1 双足 / 轮足 |
| `limxsdk::Humanoid` | `humanoid.h` | 人形机器人 (Oli) |
| `limxsdk::Wheellegged` | `wheellegged.h` | 轮腿平台 |
| `limxsdk::Tron2` | `tron2.h` | TRON2 |

所有类均继承自 `ApiBase`，提供以下方法：

```cpp
auto* robot = limxsdk::PointFoot::getInstance();
robot->init("127.0.0.1");                 // 连接机器人（仿真用 127.0.0.1）

// 基于回调的读取（订阅模式）
robot->subscribeImuData([](auto& imu) { /* 处理 IMU 数据 */ });
robot->subscribeRobotState([](auto& state) { /* state.q[i] = 关节位置 */ });
robot->subscribeDiagnosticValue([](auto& msg) { /* 标定状态、错误等 */ });

// 指令下发
robot->publishRobotCmd(cmd);              // 发送关节指令
robot->setRobotLightEffect(effect);       // 控制机器人灯效

// 工具方法
int n = robot->getMotorNumber();          // 获取电机数量
auto names = robot->getMotorNames();      // 获取电机/关节名称
```

