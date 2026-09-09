"""
@file example_tron2_brainco_hand.py

@brief Example for Tron2 brainco hand command/state interface.

This script publishes to "/brainco2/hand/cmd" and subscribes to both
"/brainco2/hand/cmd" and "/brainco2/hand/state" through limxsdk Python API.

Usage:
    python3 example_tron2_brainco_hand.py [robot_ip]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import argparse
import sys
import time

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes


PRINT_PERIOD_NS = 1_000_000_000


def _fmt_hand_msg(msg: datatypes.BraincoHandMsg) -> str:
    return "names={} pos={} vel={} current={} time={}".format(
        msg.names, msg.pos, msg.vel, msg.current, msg.time
    )


def _fmt_ctrl_mode(ctrl_mode):
    if isinstance(ctrl_mode, list):
        return ctrl_mode
    return [int(ctrl_mode[0]), int(ctrl_mode[1])]


def _build_demo_cmd() -> datatypes.BraincoHandCmd:
    cmd = datatypes.BraincoHandCmd()
    cmd.hand_type = "brainco2"
    cmd.ctrl_mode = [1, 1]

    hand_names = [
        "thumb_joint", "index_joint", "middle_joint", "ring_joint", "little_joint"
    ]
    for i in range(2):
        cmd.hand_cmd[i].names = list(hand_names)
        cmd.hand_cmd[i].vel = [0.5, 0.5, 0.5, 0.5, 0.5]
        cmd.hand_cmd[i].current = [0.2, 0.2, 0.2, 0.2, 0.2]
        cmd.hand_cmd[i].time = [0.03, 0.03, 0.03, 0.03, 0.03]

    return cmd


def _run_pub_cmd(robot, interval_sec: float):
    cmd = _build_demo_cmd()
    phase = 0
    while True:
        if phase % 2 == 0:
            target = [0.8, 0.8, 0.8, 0.8, 0.8]
            label = "close"
        else:
            target = [0.1, 0.1, 0.1, 0.1, 0.1]
            label = "open"

        for i in range(2):
            cmd.hand_cmd[i].pos = list(target)

        cmd.stamp = time.time_ns()
        ok = robot.publishBraincoHandCmd(cmd)
        print("[hand-cmd-pub] stamp={} mode={} ok={} target_pos={}".format(
            cmd.stamp, label, ok, target
        ))

        phase += 1
        time.sleep(interval_sec)


def _run_sub_cmd(robot):
    last_cmd_print_ns = [0]

    def on_hand_cmd(msg: datatypes.BraincoHandCmd):
        now_ns = msg.stamp
        if now_ns - last_cmd_print_ns[0] < PRINT_PERIOD_NS:
            return
        last_cmd_print_ns[0] = now_ns

        ctrl_mode = _fmt_ctrl_mode(msg.ctrl_mode)
        print("[hand-cmd-sub] stamp={} hand_type={} ctrl_mode={}".format(
            msg.stamp, msg.hand_type, ctrl_mode
        ))
        for i, one in enumerate(msg.hand_cmd):
            print("  cmd[{}]: {}".format(i, _fmt_hand_msg(one)))

    robot.subscribeBraincoHandCmd(on_hand_cmd)
    print("[mode=sub_cmd] subscribed /brainco2/hand/cmd, waiting messages...")
    while True:
        time.sleep(1.0)


def _run_sub_state(robot):
    last_state_print_ns = [0]

    def on_hand_state(msg: datatypes.BraincoHandState):
        now_ns = msg.stamp
        if now_ns - last_state_print_ns[0] < PRINT_PERIOD_NS:
            return
        last_state_print_ns[0] = now_ns

        ctrl_mode = _fmt_ctrl_mode(msg.ctrl_mode)
        print("[hand-state-sub] stamp={} hand_type={} ctrl_mode={}".format(
            msg.stamp, msg.hand_type, ctrl_mode
        ))
        for i, one in enumerate(msg.hand_state):
            print("  state[{}]: {}".format(i, _fmt_hand_msg(one)))

    robot.subscribeBraincoHandState(on_hand_state)
    print("[mode=sub_state] subscribed /brainco2/hand/state, waiting messages...")
    while True:
        time.sleep(1.0)


def _run_all(robot, interval_sec: float):
    last_cmd_print_ns = [0]
    last_state_print_ns = [0]

    def on_hand_cmd(msg: datatypes.BraincoHandCmd):
        now_ns = msg.stamp
        if now_ns - last_cmd_print_ns[0] < PRINT_PERIOD_NS:
            return
        last_cmd_print_ns[0] = now_ns

        ctrl_mode = _fmt_ctrl_mode(msg.ctrl_mode)
        print("[hand-cmd-sub] stamp={} hand_type={} ctrl_mode={}".format(
            msg.stamp, msg.hand_type, ctrl_mode
        ))
        for i, one in enumerate(msg.hand_cmd):
            print("  cmd[{}]: {}".format(i, _fmt_hand_msg(one)))

    def on_hand_state(msg: datatypes.BraincoHandState):
        now_ns = msg.stamp
        if now_ns - last_state_print_ns[0] < PRINT_PERIOD_NS:
            return
        last_state_print_ns[0] = now_ns

        ctrl_mode = _fmt_ctrl_mode(msg.ctrl_mode)
        print("[hand-state-sub] stamp={} hand_type={} ctrl_mode={}".format(
            msg.stamp, msg.hand_type, ctrl_mode
        ))
        for i, one in enumerate(msg.hand_state):
            print("  state[{}]: {}".format(i, _fmt_hand_msg(one)))

    robot.subscribeBraincoHandCmd(on_hand_cmd)
    robot.subscribeBraincoHandState(on_hand_state)
    print("[mode=all] pub+sub cmd+sub state enabled")
    _run_pub_cmd(robot, interval_sec)


def _parse_args():
    parser = argparse.ArgumentParser(description="Tron2 BrainCo hand interface test")
    parser.add_argument("robot_ip", nargs="?", default="10.192.1.2", help="Robot IP")
    parser.add_argument(
        "--mode",
        choices=["pub_cmd", "sub_cmd", "sub_state", "all"],
        default="all",
        help="Select a single interface test mode or run all",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="Publish interval (seconds) for pub_cmd/all modes",
    )
    return parser.parse_args()


def main():
    args = _parse_args()

    robot = Robot(RobotType.Tron2)
    if not robot.init(args.robot_ip):
        print("ERROR: robot.init failed")
        sys.exit(1)

    print("start mode={}, robot_ip={}".format(args.mode, args.robot_ip))
    if args.mode == "pub_cmd":
        _run_pub_cmd(robot, args.interval)
    elif args.mode == "sub_cmd":
        _run_sub_cmd(robot)
    elif args.mode == "sub_state":
        _run_sub_state(robot)
    else:
        _run_all(robot, args.interval)


if __name__ == "__main__":
    main()
