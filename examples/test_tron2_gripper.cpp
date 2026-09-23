/**
 * @file test_tron2_gripper.cpp
 *
 * @brief Standalone example for the Tron2 2F-gripper low-level SDK API.
 *
 * Drives and subscribes the dedicated channel "/limx/2F-gripper/cmd"
 * (size=2, [left, right]) and subscribes state feedback on
 * "/limx/2F-gripper/state". Verifies the basic
 * close / open / asymmetric grasp behaviors with the SDK's value clamping
 * (right-finger opening hard-capped to 95 to match the signaling layer).
 *
 * Usage:
 *   test_tron2_gripper [robot_ip]
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include "limxsdk/tron2.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

static void platform_sleep(int seconds) {
#ifdef _WIN32
  Sleep(seconds * 1000);
#else
  sleep(seconds);
#endif
}

int main(int argc, char *argv[]) {
  limxsdk::Tron2 *robot = limxsdk::Tron2::getInstance();

  std::string robot_ip = "10.192.1.2";  // real robot default
  if (argc > 1) {
    robot_ip = argv[1];
  }

  if (!robot->init(robot_ip)) {
    std::exit(1);
  }

  auto fmtVec = [](const std::vector<float> &v) -> std::string {
    char buf[128];
    if (v.size() >= 2) {
      snprintf(buf, sizeof(buf), "[L=%.1f, R=%.1f]", v[0], v[1]);
    } else {
      snprintf(buf, sizeof(buf), "[size=%zu]", v.size());
    }
    return std::string(buf);
  };

  robot->subscribeGripperCmd([&](const limxsdk::GripperCmdConstPtr &cmd) {
    printf("[gripper-cmd-rx] stamp=%llu opening=%s speed=%s force=%s\n",
           (unsigned long long)cmd->stamp,
           fmtVec(cmd->opening).c_str(),
           fmtVec(cmd->speed).c_str(),
           fmtVec(cmd->force).c_str());
  });

  std::atomic<uint64_t> last_print_stamp{0};
  robot->subscribeGripperState([&](const limxsdk::GripperStateConstPtr &state) {
    // 1 Hz throttle on print to avoid flooding stdout.
    uint64_t now = state->stamp;
    if (now - last_print_stamp.load() < 1000000000ULL) return;
    last_print_stamp.store(now);

    printf("[gripper-state] stamp=%llu q=%s v=%s tau=%s\n",
           (unsigned long long)state->stamp,
           fmtVec(state->q).c_str(),
           fmtVec(state->v).c_str(),
           fmtVec(state->tau).c_str());
  });

  // 简单循环演示：close -> open -> 左闭右开 -> 左开右闭
  limxsdk::GripperCmd cmd(2);
  cmd.speed = {80.0f, 80.0f};
  cmd.force = {50.0f, 50.0f};
  int step = 0;
  while (true) {
    switch (step % 4) {
      case 0:
        cmd.opening = {0.0f, 0.0f};
        printf("[gripper-cmd] close both\n");
        break;
      case 1:
        cmd.opening = {100.0f, 100.0f};  // right will be clamped to 95 by SDK
        printf("[gripper-cmd] open both (right auto-clamped to 95)\n");
        break;
      case 2:
        cmd.opening = {0.0f, 95.0f};
        printf("[gripper-cmd] left close, right open\n");
        break;
      case 3:
        cmd.opening = {95.0f, 0.0f};
        printf("[gripper-cmd] left open, right close\n");
        break;
    }
    robot->publishGripperCmd(cmd);
    platform_sleep(3);
    ++step;
  }

  return 0;
}
