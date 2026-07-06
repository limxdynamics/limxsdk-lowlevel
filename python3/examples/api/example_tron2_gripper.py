"""
@file example_tron2_gripper.py

@brief Standalone example for the Tron2 2F-gripper low-level SDK API.

Drives "/limx/2F-gripper/cmd" via Robot.publishGripperCmd() and prints the
"/limx/2F-gripper/state" feedback via Robot.subscribeGripperState().

Usage:
    python3 example_tron2_gripper.py [robot_ip]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import sys
import time

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes


def main():
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "10.192.1.2"

    robot = Robot(RobotType.Tron2)
    if not robot.init(robot_ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)

    _last_print_ns = [0]

    def on_gripper_state(state: datatypes.GripperState):
        # 1 Hz throttle to avoid spamming stdout.
        if state.stamp - _last_print_ns[0] < 1_000_000_000:
            return
        _last_print_ns[0] = state.stamp
        print("[gripper-state] stamp={} q={} v={} tau={}".format(
            state.stamp, state.q, state.v, state.tau))

    robot.subscribeGripperState(on_gripper_state)

    cmd = datatypes.GripperCmd()
    #cmd.speed = [80.0, 80.0]
    cmd.speed = [50.0, 50.0]
    cmd.force = [50.0, 50.0]

    sequence = [
        ([0.0,   0.0],  "close both"),
        ([50.0, 50.0], "open both (right auto-clamped to 95)"),
        ([0.0,   50.0], "left close, right open"),
        ([50.0,  0.0],  "left open, right close"),
    ]

    step = 0
    while True:
        opening, label = sequence[step % len(sequence)]
        cmd.opening = opening
        cmd.stamp = time.time_ns()
        print("[gripper-cmd] {} opening={}".format(label, opening))
        robot.publishGripperCmd(cmd)
        time.sleep(5.0)
        step += 1


if __name__ == "__main__":
    main()
