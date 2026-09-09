"""
@file example_tron2_generic_sub_pub_topic.py

@brief 通用订阅发布 Topic 二开示例：先 init，再查 topic，再按类型订阅 / 发布。

面向「SDK 没有专用 subscribeXxx / publishXxx，但线上已经有这条 topic」的二次开发。
消息类型来自 limxsdk.msg（limxsdk-gen 生成），不是全部 ROS 类型。

流程（复制到自己工程即可）：
  1. init(robot_ip)
  2. get_topics / get_published_topics / get_subscribed_topics
  3. query_topic_support（mirrored 且 md5_match 才能用生成类直接收发）
  4. robot.subscribe(JointState, "/motor/state", cb)
  5. robot.publish(JointCmd, "/motor/cmd")，循环发一条空命令（仅演示）
  6. robot.subscribe(TactileHandState, "/brainco2/touch/hand/state", cb)
  7. robot.publish(TactileHandCmd, "/brainco2/touch/hand/cmd")，循环发一条停止命令

/motor/cmd 与 publishRobotCmd 是同一条真机电机指令。仅作路径演示；
真机请确认当前模式，空 JointCmd（na=0）仍可能进电机控制器。

brainco2 触觉灵巧手（TactileHandCmd / TactileHandState）与 signaling 的
request_set_brainco2_touch_hand_cmd 走同一条 topic，SDK 没有专用接口，
只能走通用通道——正是本示例的适用场景。

Usage:
    python3 example_tron2_generic_sub_pub_topic.py [robot_ip]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import sys
import time

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
from limxsdk.msg import JointCmd, JointState, TactileHandCmd, TactileHandState

# hand_msgs 把手数固定为 2（[左手, 右手]），ctrl_mode / hand_cmd / hand_tactile_cmd
# 都是定长 2 的数组。手指数与触觉通道数则是 signaling 与控制器侧的事实约定，
# 消息里是变长数组，不给也能编码。
HAND_NUM = 2
FINGER_NUM = 6
TACTILE_CHANNEL_NUM = 5


def make_tactile_hand_cmd() -> TactileHandCmd:
    """ctrl_mode：0=停止 1=位置+时间 2=位置+速度 3=电流。

    两只手都填 0，这样示例即使连上真机也不会真的驱动手指。
    """
    cmd = TactileHandCmd()
    cmd.hand_type = "brainco2/hand"
    # 与 signaling 下发的同一条 topic 保持一致，便于下游按来源过滤。
    cmd.header.frame_id = "sdk"

    for h in range(HAND_NUM):
        # ctrl_mode 是定长 2 的 array.array("B")，按下标赋值而不是整体替换。
        cmd.ctrl_mode[h] = 0

        cmd.hand_cmd[h].names = [""] * FINGER_NUM
        cmd.hand_cmd[h].pos = [0.0] * FINGER_NUM
        cmd.hand_cmd[h].vel = [0.0] * FINGER_NUM
        cmd.hand_cmd[h].current = [0.0] * FINGER_NUM
        cmd.hand_cmd[h].time = [2.0] * FINGER_NUM

        # .msg 里的 bool[] 解出来是 array.array("B")，赋 list 也能编码。
        cmd.hand_tactile_cmd[h].names = [""] * TACTILE_CHANNEL_NUM
        cmd.hand_tactile_cmd[h].tactile_switch = [1] * TACTILE_CHANNEL_NUM
        cmd.hand_tactile_cmd[h].channel_reset = [0] * TACTILE_CHANNEL_NUM
        cmd.hand_tactile_cmd[h].calibration_trigger = [0] * TACTILE_CHANNEL_NUM

    return cmd


def main():
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "10.192.1.2"

    robot = Robot(RobotType.Tron2)
    if not robot.init(robot_ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)

    # get_topics 最多阻塞 duration 秒（默认 3，上界而非每次都等满）。
    # 返回 list[dict]，key 为 name / type / md5 / definition。
    # definition 经常为空：表示「未知」，不是「这个消息没有字段」；匹配也不用它。
    # MROS_DOMAIN_ID 若设成普通 token（不要用 MAC 格式），topic 名会带 /did_<id>/ 前缀。
    topics = robot.get_topics()
    print("[get_topics] {} (name / type / md5)".format(len(topics)))
    for row in topics[:8]:
        print("  {}  {}  {}  definition_empty={}".format(
            row["name"], row["type"], row["md5"],
            1 if not row["definition"] else 0))

    # 全域当前有 pub / sub 的表，不是「仅本进程」——这是 mros 的口径。
    pubs = robot.get_published_topics(0)
    subs = robot.get_subscribed_topics(0)
    print("[get_published_topics] {} (domain-wide, not this process only)".format(
        len(pubs)))
    print("[get_subscribed_topics] {} (domain-wide, not this process only)".format(
        len(subs)))

    # 只有 mirrored 与 md5_match 都为 True，才能用生成类直接 subscribe / publish。
    # TYPE 本地没有：mirrored=False, md5_match=False。
    # TYPE 有但 MD5 不一致：mirrored=True, md5_match=False（名字熟、线格式不是这一版）。
    support = robot.query_topic_support(0)
    print("[query_topic_support] {}".format(len(support)))
    for row in support[:8]:
        print("  {}  {}  mirrored={}  md5_match={}".format(
            row["name"], row["type"], row["mirrored"], row["md5_match"]))

    # 回调拿到的是已经 decode 好的 JointState 对象，不是 memoryview。
    # 真机 /motor/state 频率很高，这里隔约 1s 打一行。
    last_print_ns = [0]

    def on_joint_state(msg: JointState):
        stamp = msg.header.stamp.to_nsec()
        if stamp - last_print_ns[0] < 1_000_000_000:
            return
        last_print_ns[0] = stamp

        line = "[JointState] seq={} names={} q={} na={}".format(
            msg.header.seq, len(msg.names), len(msg.q), msg.na)
        if len(msg.q) > 0:
            line += "  q[0]={:.3f}".format(msg.q[0])
        if msg.names:
            line += "  name[0]={}".format(msg.names[0])
        print(line)

    sub = robot.subscribe(JointState, "/motor/state", on_joint_state)
    if not sub.valid:
        print('ERROR: subscribe("/motor/state") failed.')
        sys.exit(1)

    # advertise 一次（Python 门面叫 publish），循环里再 pub.publish(msg)。
    # 警告：仅演示。真机请确认模式，空命令仍可能进电机。
    pub = robot.publish(JointCmd, "/motor/cmd")
    if not pub.valid:
        print('ERROR: advertise("/motor/cmd") failed.')
        sys.exit(1)

    # brainco2 触觉手反馈：TactileHandState 比 HandState 多一组 TactileState[2]，
    # 每个通道有 normal_force / tangential_force / direction_angle /
    # approximate_value / tactile_state 五组变长数组（约定长度 5）。
    last_tactile_print_ns = [0]

    def on_tactile_hand_state(msg: TactileHandState):
        stamp = msg.header.stamp.to_nsec()
        if stamp - last_tactile_print_ns[0] < 1_000_000_000:
            return
        last_tactile_print_ns[0] = stamp

        left = msg.hand_tactile_state[0]
        line = ("[TactileHandState] seq={} hand_type={} mode=[{},{}] "
                "L.fingers={} L.channels={}".format(
                    msg.header.seq, msg.hand_type,
                    msg.ctrl_mode[0], msg.ctrl_mode[1],
                    len(msg.hand_state[0].pos), len(left.normal_force)))
        if len(left.normal_force) > 0:
            line += "  L.normal_force[0]={:.3f}".format(left.normal_force[0])
        if len(left.tactile_state) > 0:
            line += "  L.tactile_state[0]={}".format(left.tactile_state[0])
        print(line)

    tactile_sub = robot.subscribe(TactileHandState,
                                  "/brainco2/touch/hand/state",
                                  on_tactile_hand_state)
    if not tactile_sub.valid:
        print('ERROR: subscribe("/brainco2/touch/hand/state") failed.')
        sys.exit(1)

    # 与 signaling 的 request_set_brainco2_touch_hand_cmd 是同一条 topic，
    # 两边同时下发会互相覆盖；调试时先确认没有别的下发方。
    tactile_pub = robot.publish(TactileHandCmd, "/brainco2/touch/hand/cmd")
    if not tactile_pub.valid:
        print('ERROR: advertise("/brainco2/touch/hand/cmd") failed.')
        sys.exit(1)

    while True:
        cmd = JointCmd()
        cmd.na = 0
        pub.publish(cmd)

        tactile_pub.publish(make_tactile_hand_cmd())
        time.sleep(1.0)


if __name__ == "__main__":
    main()
