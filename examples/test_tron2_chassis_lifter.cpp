/**
 * @file test_tron2_chassis_lifter.cpp
 *
 * @brief Standalone example for the Tron2 chassis / lifter state subscriptions.
 *
 * Subscribes read-only feedback channels:
 *   - "/chassis_state" (std_msgs/Float32MultiArray) -> ChassisState (raw data[])
 *   - "/lifter/state"  (controller_msgs/JointState)  -> RobotState (q/dq/tau)
 *
 * Usage:
 *   test_tron2_chassis_lifter [robot_ip]
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
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

static std::string fmtVec(const std::vector<float> &v) {
  std::string s = "[";
  for (std::size_t i = 0; i < v.size(); ++i) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.3f%s", v[i], (i + 1 < v.size()) ? ", " : "");
    s += buf;
  }
  s += "]";
  return s;
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

  std::atomic<uint64_t> last_chassis_print{0};
  robot->subscribeChassisState([&](const limxsdk::ChassisStateConstPtr &state) {
    // 1 Hz throttle on print to avoid flooding stdout.
    uint64_t now = state->stamp;
    if (now - last_chassis_print.load() < 1000000000ULL) return;
    last_chassis_print.store(now);

    printf("[chassis-state] stamp=%llu data=%s\n",
           (unsigned long long)state->stamp,
           fmtVec(state->data).c_str());
  });

  std::atomic<uint64_t> last_lifter_print{0};
  robot->subscribeLifterState([&](const limxsdk::RobotStateConstPtr &state) {
    uint64_t now = state->stamp;
    if (now - last_lifter_print.load() < 1000000000ULL) return;
    last_lifter_print.store(now);

    printf("[lifter-state] stamp=%llu q=%s dq=%s tau=%s\n",
           (unsigned long long)state->stamp,
           fmtVec(state->q).c_str(),
           fmtVec(state->dq).c_str(),
           fmtVec(state->tau).c_str());
  });

  printf("Subscribed /chassis_state and /lifter/state. Ctrl+C to exit.\n");
  while (true) {
    platform_sleep(1);
  }

  return 0;
}
