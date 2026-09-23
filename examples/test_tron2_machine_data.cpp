/**
 * @file test_tron2_machine_data.cpp
 *
 * @brief Standalone example for the Tron2 machine-data low-level SDK APIs.
 *
 * Subscribes every read-only machine-data channel exposed for secondary
 * development and prints one throttled line per channel:
 *   - gripper command read-back ("/limx/2F-gripper/cmd")
 *   - lifting-column feedback   ("/lifter/state", raw controller units)
 *   - lifting-column status     ("/lifter/status", mm + fault flags)
 *   - wheeled-base motion       ("/chassis/vel/state")
 *   - measured arm EE pose      ("/arm/ee_pose_state")
 *   - dexterous-hand feedback   ("/brainco2/hand/state")
 *   - VR head-set / controllers ("/vr_cmd")
 *   - tele-operation status     ("/diagnostics_value", "TeleOperation")
 *   - one named diagnostic      ("/diagnostics_value", --diag=<name>)
 *
 * Each subscribe* call creates its MROS subscriber on first registration only,
 * so channels left unsubscribed cost nothing. Callbacks run on the MROS
 * dispatch thread and receive a freshly allocated snapshot, so the payload
 * stays valid for as long as the caller keeps the shared_ptr.
 *
 * The write channels are opt-in, because they move real hardware:
 *   --send-hand-cmd   one open-hand command on "/brainco2/hand/cmd"
 *   --drive-chassis   5 s of slow forward motion on "/sdk_cmd_vel" (10 Hz)
 *   --drive-lifter    5 s of slow raise then a stop frame on "/sdk_lifter_vel"
 *   --lifter-pos=<mm> 5 s of position streaming on "/lifter/pos/cmd"
 * Both lifter and chassis inputs are STREAMING: the base stops on its own
 * roughly 300 ms after the frames stop, which is why the loops below keep
 * publishing rather than sending one frame.
 *
 *   --urdf            blocking HTTP fetch of the URDF, prints its size
 *
 * Usage:
 *   test_tron2_machine_data [robot_ip] [options]
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
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

static void platform_sleep_ms(int milliseconds) {
#ifdef _WIN32
  Sleep(milliseconds);
#else
  usleep(milliseconds * 1000);
#endif
}

// 1 Hz print throttle, one independent slot per channel.
static bool shouldPrint(std::atomic<uint64_t> &last, uint64_t stamp) {
  if (stamp - last.load() < 1000000000ULL) return false;
  last.store(stamp);
  return true;
}

static std::string fmtVec(const std::vector<float> &v, size_t max_n = 4) {
  std::string out = "[";
  for (size_t i = 0; i < v.size() && i < max_n; i++) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%s%.3f", i ? ", " : "", v[i]);
    out += buf;
  }
  if (v.size() > max_n) out += ", ...";
  out += "]";
  return out;
}

int main(int argc, char *argv[]) {
  limxsdk::Tron2 *robot = limxsdk::Tron2::getInstance();

  std::string robot_ip = "10.192.1.2";  // real robot default
  bool send_hand_cmd = false;
  bool drive_chassis = false;
  bool drive_lifter = false;
  bool fetch_urdf = false;
  bool lifter_pos_set = false;
  float lifter_pos_mm = 0.0f;
  std::string diag_name;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--send-hand-cmd") == 0) {
      send_hand_cmd = true;
    } else if (strcmp(argv[i], "--drive-chassis") == 0) {
      drive_chassis = true;
    } else if (strcmp(argv[i], "--drive-lifter") == 0) {
      drive_lifter = true;
    } else if (strcmp(argv[i], "--urdf") == 0) {
      fetch_urdf = true;
    } else if (strncmp(argv[i], "--lifter-pos=", 13) == 0) {
      lifter_pos_mm = (float)atof(argv[i] + 13);
      lifter_pos_set = true;
    } else if (strncmp(argv[i], "--diag=", 7) == 0) {
      diag_name = argv[i] + 7;
    } else if (argv[i][0] != '-') {
      robot_ip = argv[i];
    }
  }

  if (!robot->init(robot_ip)) {
    std::exit(1);
  }

  static std::atomic<uint64_t> t_grip{0}, t_lift{0}, t_lstatus{0}, t_chas{0};
  static std::atomic<uint64_t> t_arm{0}, t_hand{0}, t_vr{0};

  robot->subscribeGripperCmd([](const limxsdk::GripperCmdConstPtr &cmd) {
    if (!shouldPrint(t_grip, cmd->stamp)) return;
    printf("[gripper-cmd]  opening=%s speed=%s force=%s\n",
           fmtVec(cmd->opening).c_str(), fmtVec(cmd->speed).c_str(),
           fmtVec(cmd->force).c_str());
  });

  robot->subscribeLifterState([](const limxsdk::LifterStateConstPtr &s) {
    if (!shouldPrint(t_lift, s->stamp)) return;
    printf("[lifter-state] na=%u q=%s v=%s tau=%s\n",
           s->na, fmtVec(s->q).c_str(), fmtVec(s->v).c_str(),
           fmtVec(s->tau).c_str());
  });

  robot->subscribeLifterStatus([](const limxsdk::LifterStatusConstPtr &s) {
    if (!shouldPrint(t_lstatus, s->stamp)) return;
    if (!s->valid) {
      printf("[lifter-status] short frame (n=%zu), fields not decoded\n",
             s->data.size());
      return;
    }
    // Spell out the flags that explain why a lifter command would be refused or
    // altered, since that is the whole point of reading this channel.
    std::string notes;
    if (s->flags & limxsdk::LifterStatus::NOT_CALIBRATED) notes += " NOT_CALIBRATED";
    if (s->flags & limxsdk::LifterStatus::STATE_NOT_READY) notes += " STATE_NOT_READY";
    if (s->flags & limxsdk::LifterStatus::WATCHDOG_LATCHED) notes += " WATCHDOG_LATCHED";
    if (s->flags & limxsdk::LifterStatus::SPEED_FALLBACK) notes += " SPEED_FALLBACK";
    if (s->flags & limxsdk::LifterStatus::CMD_CLAMPED) notes += " CMD_CLAMPED";
    if (s->flags & limxsdk::LifterStatus::FRAME_IGNORED) notes += " FRAME_IGNORED";
    printf("[lifter-status] pos=%.1fmm vel=%.1fmm/s flags=0x%03x owner=%d task=%d%s\n",
           s->position_mm, s->velocity_mm_s, s->flags, s->owner, s->task,
           notes.c_str());
  });

  robot->subscribeChassisState([](const limxsdk::ChassisStateConstPtr &s) {
    if (!shouldPrint(t_chas, s->stamp)) return;
    printf("[chassis]      linear=%.3f angular=%.3f steering=%.3f (raw n=%zu)\n",
           s->linear_velocity, s->angular_velocity, s->steering_angle,
           s->data.size());
  });

  robot->subscribeArmEePose([](const limxsdk::ArmEePoseConstPtr &p) {
    if (!shouldPrint(t_arm, p->stamp)) return;
    if (!p->valid) {
      printf("[arm-ee-pose]  short frame (n=%zu), fields not decoded\n",
             p->data.size());
      return;
    }
    printf("[arm-ee-pose]  L pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)\n"
           "               R pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)\n",
           p->left_position[0], p->left_position[1], p->left_position[2],
           p->left_quat[0], p->left_quat[1], p->left_quat[2], p->left_quat[3],
           p->right_position[0], p->right_position[1], p->right_position[2],
           p->right_quat[0], p->right_quat[1], p->right_quat[2], p->right_quat[3]);
  });

  robot->subscribeDexHandState([](const limxsdk::DexHandStateConstPtr &s) {
    if (!shouldPrint(t_hand, s->stamp)) return;
    std::string body;
    for (size_t h = 0; h < s->hands.size(); h++) {
      body += "\n               ";
      body += (h == 0 ? "L pos=" : "R pos=");
      body += fmtVec(s->hands[h].pos, 6);
      body += " current=" + fmtVec(s->hands[h].current, 6);
    }
    printf("[dex-hand]     type=%s mode=[%u, %u]%s\n", s->hand_type.c_str(),
           s->ctrl_mode.size() > 0 ? s->ctrl_mode[0] : 0,
           s->ctrl_mode.size() > 1 ? s->ctrl_mode[1] : 0, body.c_str());
  });

  robot->subscribeVrState([](const limxsdk::VrStateConstPtr &v) {
    if (!shouldPrint(t_vr, v->stamp)) return;
    printf("[vr-state]     L js=(%.2f, %.2f) trig=%.2f grip=%.2f X=%d Y=%d\n"
           "               R js=(%.2f, %.2f) trig=%.2f grip=%.2f A=%d B=%d\n",
           v->left_joystick[0], v->left_joystick[1], v->left_trigger,
           v->left_grip, (int)v->button_x, (int)v->button_y,
           v->right_joystick[0], v->right_joystick[1], v->right_trigger,
           v->right_grip, (int)v->button_a, (int)v->button_b);
  });

  robot->subscribeTeleopState([](const limxsdk::TeleopStateConstPtr &t) {
    // Not throttled: these frames are edge published and each one matters.
    printf("[teleop]       takeover=%s healthy=%d level=%d code=%d frame=%s msg=%s\n",
           t->takeover_known ? (t->takeover_active ? "yes" : "no") : "unknown",
           (int)t->healthy, t->level, t->code, t->frame_id.c_str(),
           t->message.c_str());
  });

  if (!diag_name.empty()) {
    robot->subscribeDiagnosticByName(diag_name, [](const limxsdk::DiagnosticEntryConstPtr &d) {
      printf("[diag]         name=%s frame=%s level=%d code=%d msg=%s\n",
             d->name.c_str(), d->frame_id.c_str(), d->level, d->code,
             d->message.c_str());
    });
    printf("Filtering /diagnostics_value for name=\"%s\".\n", diag_name.c_str());
  }

  if (send_hand_cmd) {
    limxsdk::DexHandCmd cmd;
    cmd.ctrl_mode = {2, 2};  // position-velocity control on both hands
    for (size_t h = 0; h < cmd.hands.size(); h++) {
      cmd.hands[h].pos.assign(6, 0.0f);
      cmd.hands[h].vel.assign(6, 10.0f);
    }
    printf("[dex-hand-cmd] sending open-hand command (mode=2)\n");
    if (!robot->publishDexHandCmd(cmd)) {
      printf("[dex-hand-cmd] publish rejected\n");
    }
  }

  if (fetch_urdf) {
    std::string urdf;
    if (robot->getRobotDescription(urdf)) {
      printf("[urdf]         fetched %zu bytes, starts with: %.60s\n", urdf.size(),
             urdf.c_str());
    } else {
      printf("[urdf]         fetch failed\n");
    }
  }

  // 10 Hz for 5 s. Both interfaces are streaming, so the loop is the command:
  // stopping the loop is what stops the motion.
  const int kStreamHz = 10;
  const int kStreamSeconds = 5;
  const int kStreamFrames = kStreamHz * kStreamSeconds;

  if (drive_chassis) {
    printf("[chassis-cmd]  streaming linear_x=0.2 for %d s at %d Hz\n", kStreamSeconds,
           kStreamHz);
    for (int i = 0; i < kStreamFrames; i++) {
      if (!robot->publishChassisTwist(0.2f, 0.0f)) {
        printf("[chassis-cmd]  publish rejected\n");
        break;
      }
      platform_sleep_ms(1000 / kStreamHz);
    }
    // Explicit zero frame instead of relying on the watchdog.
    robot->publishChassisTwist(0.0f, 0.0f);
    printf("[chassis-cmd]  sent stop frame\n");
  }

  if (drive_lifter) {
    printf("[lifter-cmd]   streaming v=+20 mm/s for %d s at %d Hz\n", kStreamSeconds,
           kStreamHz);
    for (int i = 0; i < kStreamFrames; i++) {
      if (!robot->publishLifterVel(20.0f)) {
        printf("[lifter-cmd]   publish rejected\n");
        break;
      }
      platform_sleep_ms(1000 / kStreamHz);
    }
    // v=0 latches the measured position and releases the streaming slot.
    robot->publishLifterVel(0.0f);
    printf("[lifter-cmd]   sent v=0 stop frame\n");
  }

  if (lifter_pos_set) {
    printf("[lifter-cmd]   streaming target=%.1f mm limit=30 mm/s for %d s\n",
           lifter_pos_mm, kStreamSeconds);
    for (int i = 0; i < kStreamFrames; i++) {
      if (!robot->publishLifterPos(lifter_pos_mm, 30.0f)) {
        printf("[lifter-cmd]   publish rejected\n");
        break;
      }
      platform_sleep_ms(1000 / kStreamHz);
    }
    // Position streaming has no "0 = stop", so stop through the velocity topic.
    robot->publishLifterVel(0.0f);
    printf("[lifter-cmd]   stopped via publishLifterVel(0)\n");
  }

  printf("Listening for machine data; press Ctrl-C to stop.\n");
  while (true) {
    platform_sleep(1);
  }

  return 0;
}
