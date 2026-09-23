#include <gtest/gtest.h>
#include "limxsdk/humanoid.h"
#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"
#include <string>

std::string robot_ip = "127.0.0.1";

// 测试 Humanoid 类的初始化
TEST(HumanoidTest, InitTest) {
    limxsdk::Humanoid* humanoid = limxsdk::Humanoid::getInstance();
    bool result = humanoid->init(robot_ip);
    EXPECT_TRUE(result);
}

// 测试发布机器人命令
TEST(HumanoidTest, PublishRobotCmdTest) {
    limxsdk::Humanoid* humanoid = limxsdk::Humanoid::getInstance();
    limxsdk::RobotCmd cmd;
    uint32_t motorNumber = humanoid->getMotorNumber();
    cmd.mode.resize(motorNumber, 0);
    cmd.tau.resize(motorNumber, 0);
    cmd.q.resize(motorNumber, 0);
    cmd.dq.resize(motorNumber, 0);
    cmd.Kd.resize(motorNumber, 0);
    cmd.Kp.resize(motorNumber, 0);
    cmd.stamp = 0;

    bool result = humanoid->publishRobotCmd(cmd);
    EXPECT_TRUE(result);
}

// 测试订阅机器人状态
TEST(HumanoidTest, SubscribeRobotStateTest) {
    
    limxsdk::Humanoid* humanoid = limxsdk::Humanoid::getInstance();
    auto cb = [](const limxsdk::RobotStateConstPtr& state) {
        // 简单的回调函数
    };
    humanoid->subscribeRobotState(cb);
    // 这里无法直接验证回调是否被调用，只是验证调用不会崩溃

    EXPECT_TRUE(true);
}

// 测试订阅 IMU 数据
TEST(HumanoidTest, SubscribeImuDataTest) {
    limxsdk::Humanoid* humanoid = limxsdk::Humanoid::getInstance();
    auto cb = [](const limxsdk::ImuDataConstPtr& imu) {
        // 简单的回调函数
    };
    humanoid->subscribeImuData(cb);
    // 这里无法直接验证回调是否被调用，只是验证调用不会崩溃

    EXPECT_TRUE(true);
}

// 测试订阅传感器 Joy 数据
TEST(HumanoidTest, SubscribeSensorJoyTest) {
    limxsdk::Humanoid* humanoid = limxsdk::Humanoid::getInstance();

    auto cb = [](const limxsdk::SensorJoyConstPtr& joy) {
        // 简单的回调函数
    };
    humanoid->subscribeSensorJoy(cb);
    // 这里无法直接验证回调是否被调用，只是验证调用不会崩溃

    EXPECT_TRUE(true);
}

// 测试订阅诊断值
TEST(HumanoidTest, SubscribeDiagnosticValueTest) {
    limxsdk::Humanoid* humanoid = limxsdk::Humanoid::getInstance();

    auto cb = [](const limxsdk::DiagnosticValueConstPtr& diag) {
        // 简单的回调函数
    };
    humanoid->subscribeDiagnosticValue(cb);
    // 这里无法直接验证回调是否被调用，只是验证调用不会崩溃

    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}