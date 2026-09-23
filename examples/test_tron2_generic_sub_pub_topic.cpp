/**
 * @file test_tron2_generic_sub_pub_topic.cpp
 *
 * @brief 通用订阅发布 Topic 二开示例：先 init，再查 topic，再按类型订阅 / 发布。
 *
 * 面向「SDK 没有专用 subscribeXxx / publishXxx，但线上已经有这条 topic」的二次开发。
 * 消息类型来自 limxsdk-gen 生成的镜像（limxsdk/msg/<pkg>/<Name>.h），不是全部 ROS 类型。
 *
 * 流程（复制到自己工程即可）：
 *   1. init(robot_ip)
 *   2. getTopics / getPublishedTopics / getSubscribedTopics
 *   3. queryTopicSupport（mirrored && md5_match 才能用生成类直接收发）
 *   4. subscribe<JointState>("/motor/state")
 *   5. advertise<JointCmd>("/motor/cmd")，循环发一条空命令（仅演示）
 *   6. subscribe<TactileHandState>("/brainco2/touch/hand/state")
 *   7. advertise<TactileHandCmd>("/brainco2/touch/hand/cmd")，循环发一条停止命令
 *
 * /motor/cmd 与 publishRobotCmd 是同一条真机电机指令。仅作路径演示；
 * 真机请确认当前模式，空 JointCmd（na=0）仍可能进电机控制器。
 *
 * brainco2 触觉灵巧手（TactileHandCmd / TactileHandState）与 signaling 的
 * request_set_brainco2_touch_hand_cmd 走同一条 topic，SDK 没有专用接口，
 * 只能走通用通道——正是本示例的适用场景。
 *
 * Usage:
 *   test_tron2_generic_sub_pub_topic [robot_ip]
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "limxsdk/msg/controller_msgs/JointCmd.h"
#include "limxsdk/msg/controller_msgs/JointState.h"
#include "limxsdk/msg/hand_msgs/TactileHandCmd.h"
#include "limxsdk/msg/hand_msgs/TactileHandState.h"
#include "limxsdk/topic_channel.h"
#include "limxsdk/tron2.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

using JointCmd = limxsdk::msg::controller_msgs::JointCmd;
using JointState = limxsdk::msg::controller_msgs::JointState;
using TactileHandCmd = limxsdk::msg::hand_msgs::TactileHandCmd;
using TactileHandState = limxsdk::msg::hand_msgs::TactileHandState;
using TactileState = limxsdk::msg::hand_msgs::TactileState;

// hand_msgs 把手数固定为 2（[左手, 右手]），ctrl_mode / hand_cmd / hand_tactile_cmd
// 都是定长 2 的数组。手指数与触觉通道数则是 signaling 与控制器侧的事实约定，
// 消息里是变长数组，不给也能编码。
static const std::size_t kHandNum = 2;
static const std::size_t kFingerNum = 6;
static const std::size_t kTactileChannelNum = 5;

static void platform_sleep(int seconds) {
#ifdef _WIN32
  Sleep(seconds * 1000);
#else
  sleep(seconds);
#endif
}

// ctrl_mode：0=停止 1=位置+时间 2=位置+速度 3=电流。
// 两只手都填 0，这样示例即使连上真机也不会真的驱动手指。
static TactileHandCmd make_tactile_hand_cmd() {
  TactileHandCmd cmd;
  cmd.hand_type = "brainco2/hand";
  // 与 signaling 下发的同一条 topic 保持一致，便于下游按来源过滤。
  cmd.header.frame_id = "sdk";

  for (std::size_t h = 0; h < kHandNum; ++h) {
    cmd.ctrl_mode[h] = 0;

    cmd.hand_cmd[h].names.assign(kFingerNum, "");
    cmd.hand_cmd[h].pos.assign(kFingerNum, 0.0f);
    cmd.hand_cmd[h].vel.assign(kFingerNum, 0.0f);
    cmd.hand_cmd[h].current.assign(kFingerNum, 0.0f);
    cmd.hand_cmd[h].time.assign(kFingerNum, 2.0f);

    // .msg 里的 bool[] 生成的是 std::vector<uint8_t>，不是 std::vector<bool>。
    cmd.hand_tactile_cmd[h].names.assign(kTactileChannelNum, "");
    cmd.hand_tactile_cmd[h].tactile_switch.assign(kTactileChannelNum, 1);
    cmd.hand_tactile_cmd[h].channel_reset.assign(kTactileChannelNum, 0);
    cmd.hand_tactile_cmd[h].calibration_trigger.assign(kTactileChannelNum, 0);
  }
  return cmd;
}

int main(int argc, char *argv[]) {
  limxsdk::Tron2 *robot = limxsdk::Tron2::getInstance();

  std::string robot_ip = "10.192.1.2";
  if (argc > 1) {
    robot_ip = argv[1];
  }

  if (!robot->init(robot_ip)) {
    printf("ERROR: init(\"%s\") failed.\n", robot_ip.c_str());
    std::exit(1);
  }

  // getTopics 最多阻塞 duration_sec 秒（默认 3，上界而非每次都等满）。
  // definition 经常为空：表示「未知」，不是「这个消息没有字段」；匹配也不用它。
  // MROS_DOMAIN_ID 若设成普通 token（不要用 MAC 格式），topic 名会带 /did_<id>/ 前缀。
  std::vector<limxsdk::channel::TopicInfo> topics = limxsdk::channel::getTopics();
  printf("[getTopics] %zu (name / type / md5)\n", topics.size());
  for (std::size_t i = 0; i < topics.size() && i < 8; ++i) {
    const limxsdk::channel::TopicInfo &t = topics[i];
    printf("  %s  %s  %s  definition_empty=%d\n",
           t.name.c_str(), t.type.c_str(), t.md5.c_str(),
           t.definition.empty() ? 1 : 0);
  }

  // 全域当前有 pub / sub 的表，不是「仅本进程」——这是 mros 的口径。
  std::vector<limxsdk::channel::TopicInfo> pubs =
      limxsdk::channel::getPublishedTopics(0);
  std::vector<limxsdk::channel::TopicInfo> subs =
      limxsdk::channel::getSubscribedTopics(0);
  printf("[getPublishedTopics] %zu (domain-wide, not this process only)\n",
         pubs.size());
  printf("[getSubscribedTopics] %zu (domain-wide, not this process only)\n",
         subs.size());

  // 只有 mirrored 与 md5_match 都为 true，才能用生成类直接 subscribe / advertise。
  // TYPE 本地没有：mirrored=false, md5_match=false。
  // TYPE 有但 MD5 不一致：mirrored=true, md5_match=false（名字熟、线格式不是这一版）。
  std::vector<limxsdk::channel::TopicSupport> support =
      limxsdk::channel::queryTopicSupport(0);
  printf("[queryTopicSupport] %zu\n", support.size());
  for (std::size_t i = 0; i < support.size() && i < 8; ++i) {
    const limxsdk::channel::TopicSupport &row = support[i];
    printf("  %s  %s  mirrored=%d  md5_match=%d\n",
           row.name.c_str(), row.type.c_str(),
           (int)row.mirrored, (int)row.md5_match);
  }

  // 回调拿到的是每帧新建的 decoded 对象（shared_ptr<const JointState>），可以存进队列。
  // 真机 /motor/state 频率很高，这里隔约 1s 打一行。
  std::atomic<uint64_t> last_print_ns{0};
  auto sub = limxsdk::channel::subscribe<JointState>(
      "/motor/state", [&](const std::shared_ptr<const JointState> &msg) {
        uint64_t now = (uint64_t)msg->header.stamp.sec * 1000000000ULL +
                       (uint64_t)msg->header.stamp.nsec;
        if (now - last_print_ns.load() < 1000000000ULL) {
          return;
        }
        last_print_ns.store(now);

        printf("[JointState] seq=%u names=%zu q=%zu na=%u",
               msg->header.seq, msg->names.size(), msg->q.size(), msg->na);
        if (!msg->q.empty()) {
          printf("  q[0]=%.3f", msg->q[0]);
        }
        if (!msg->names.empty()) {
          printf("  name[0]=%s", msg->names[0].c_str());
        }
        printf("\n");
      });
  if (!sub.valid()) {
    printf("ERROR: subscribe(\"/motor/state\") failed.\n");
    std::exit(1);
  }

  // advertise 一次，循环里 publish；不要每帧重新 advertise。
  // 警告：仅演示。真机请确认模式，空命令仍可能进电机。
  auto pub = limxsdk::channel::advertise<JointCmd>("/motor/cmd");
  if (!pub.valid()) {
    printf("ERROR: advertise(\"/motor/cmd\") failed.\n");
    std::exit(1);
  }

  // brainco2 触觉手反馈：TactileHandState 比 HandState 多一组 TactileState[2]，
  // 每个通道有 normal_force / tangential_force / direction_angle /
  // approximate_value / tactile_state 五组变长数组（约定长度 5）。
  std::atomic<uint64_t> last_tactile_print_ns{0};
  auto tactile_sub = limxsdk::channel::subscribe<TactileHandState>(
      "/brainco2/touch/hand/state",
      [&](const std::shared_ptr<const TactileHandState> &msg) {
        uint64_t now = (uint64_t)msg->header.stamp.sec * 1000000000ULL +
                       (uint64_t)msg->header.stamp.nsec;
        if (now - last_tactile_print_ns.load() < 1000000000ULL) {
          return;
        }
        last_tactile_print_ns.store(now);

        const TactileState &left = msg->hand_tactile_state[0];
        printf("[TactileHandState] seq=%u hand_type=%s mode=[%u,%u] "
               "L.fingers=%zu L.channels=%zu",
               msg->header.seq, msg->hand_type.c_str(),
               (unsigned)msg->ctrl_mode[0], (unsigned)msg->ctrl_mode[1],
               msg->hand_state[0].pos.size(), left.normal_force.size());
        if (!left.normal_force.empty()) {
          printf("  L.normal_force[0]=%.3f", left.normal_force[0]);
        }
        if (!left.tactile_state.empty()) {
          printf("  L.tactile_state[0]=%u", (unsigned)left.tactile_state[0]);
        }
        printf("\n");
      });
  if (!tactile_sub.valid()) {
    printf("ERROR: subscribe(\"/brainco2/touch/hand/state\") failed.\n");
    std::exit(1);
  }

  // 与 signaling 的 request_set_brainco2_touch_hand_cmd 是同一条 topic，
  // 两边同时下发会互相覆盖；调试时先确认没有别的下发方。
  auto tactile_pub =
      limxsdk::channel::advertise<TactileHandCmd>("/brainco2/touch/hand/cmd");
  if (!tactile_pub.valid()) {
    printf("ERROR: advertise(\"/brainco2/touch/hand/cmd\") failed.\n");
    std::exit(1);
  }

  while (true) {
    JointCmd cmd;
    cmd.na = 0;
    pub.publish(cmd);

    tactile_pub.publish(make_tactile_hand_cmd());
    platform_sleep(1);
  }

  return 0;
}
