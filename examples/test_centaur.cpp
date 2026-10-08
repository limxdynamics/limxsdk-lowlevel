/**
 * @file test_centaur.cpp
 *
 * @brief Standalone example for the Centaur composed upper / lower body API.
 *
 * Centaur is two independent motor + IMU channels behind one instance:
 *   - lower body  "/ImuData"            "/motor/state"            "/motor/cmd"
 *   - upper body  "/did_upbody/ImuData" "/did_upbody/motor/state" "/did_upbody/motor/cmd"
 *
 * The unqualified inherited API (getMotorNumber / publishRobotCmd / ...) is a
 * lower-body alias, so always use the explicit *LowerBody* / *UpperBody* calls
 * when both halves are in play.
 *
 * About RobotCmd::motor_names: by default (setIgnoreMotorNames(true)) the SDK
 * ignores the field completely and publishes nameless entries, which matches
 * the joint state robots currently report. It is filled in below anyway so the
 * example still works after setIgnoreMotorNames(false), where each half must
 * carry its own names and Centaur compares them against that half's latest
 * RobotState element for element, order included.
 *
 * This example only ever sends zero torque / zero gain commands, so it holds
 * position rather than moving the robot.
 *
 * Usage:
 *   test_centaur [robot_ip]
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>
#include "limxsdk/centaur.h"

static std::string fmtNames(const std::vector<std::string> &names) {
  std::string s = "[";
  for (std::size_t i = 0; i < names.size(); ++i) {
    s += names[i];
    if (i + 1 < names.size()) s += ", ";
  }
  s += "]";
  return s;
}

// 1 Hz throttle so the callbacks do not flood stdout.
static bool throttle(std::atomic<uint64_t> &last, uint64_t stamp) {
  if (stamp - last.load() < 1000000000ULL) return false;
  last.store(stamp);
  return true;
}

static void subscribeBody(limxsdk::Centaur *robot, bool upper) {
  const char *tag = upper ? "upper" : "lower";

  static std::atomic<uint64_t> last_state_lower{0}, last_state_upper{0};
  static std::atomic<uint64_t> last_imu_lower{0}, last_imu_upper{0};
  std::atomic<uint64_t> &last_state = upper ? last_state_upper : last_state_lower;
  std::atomic<uint64_t> &last_imu = upper ? last_imu_upper : last_imu_lower;

  auto state_cb = [tag, &last_state](const limxsdk::RobotStateConstPtr &state) {
    if (!throttle(last_state, state->stamp)) return;
    printf("[%s-state] motors=%lu names=%s\n",
           tag,
           (unsigned long)state->motor_names.size(),
           fmtNames(state->motor_names).c_str());
  };

  auto imu_cb = [tag, &last_imu](const limxsdk::ImuDataConstPtr &imu) {
    if (!throttle(last_imu, imu->stamp)) return;
    printf("[%s-imu] acc=[%.3f, %.3f, %.3f] gyro=[%.3f, %.3f, %.3f]\n",
           tag,
           imu->acc[0], imu->acc[1], imu->acc[2],
           imu->gyro[0], imu->gyro[1], imu->gyro[2]);
  };

  if (upper) {
    robot->subscribeUpperBodyRobotState(state_cb);
    robot->subscribeUpperBodyImuData(imu_cb);
  } else {
    robot->subscribeLowerBodyRobotState(state_cb);
    robot->subscribeLowerBodyImuData(imu_cb);
  }
}

// Sends one zero-effort command to the requested half.
static bool holdPosition(limxsdk::Centaur *robot, bool upper) {
  // Blocks until that half has delivered its first RobotState.
  std::vector<std::string> names =
      upper ? robot->getUpperBodyMotorNames() : robot->getLowerBodyMotorNames();
  const std::size_t n = names.size();

  limxsdk::RobotCmd cmd;
  cmd.stamp = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
  cmd.mode.assign(n, 0);
  cmd.q.assign(n, 0.0f);
  cmd.dq.assign(n, 0.0f);
  cmd.tau.assign(n, 0.0f);
  cmd.Kp.assign(n, 0.0f);
  cmd.Kd.assign(n, 0.0f);
  cmd.parallel_solve_required.assign(n, false);
  // Inert under the default ignore policy; kept so strict mode also works.
  cmd.motor_names = names;

  return upper ? robot->publishUpperBodyRobotCmd(cmd)
               : robot->publishLowerBodyRobotCmd(cmd);
}

int main(int argc, char *argv[]) {
  limxsdk::Centaur *robot = limxsdk::Centaur::getInstance();

  std::string robot_ip = "10.192.1.2";  // real robot default
  if (argc > 1) {
    robot_ip = argv[1];
  }

  if (!robot->init(robot_ip)) {
    std::exit(1);
  }

  subscribeBody(robot, false);
  subscribeBody(robot, true);

  printf("Waiting for the first RobotState of both bodies...\n");
  printf("lower motors=%u, upper motors=%u\n",
         robot->getLowerBodyMotorNumber(),
         robot->getUpperBodyMotorNumber());

  while (true) {
    holdPosition(robot, false);
    holdPosition(robot, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
