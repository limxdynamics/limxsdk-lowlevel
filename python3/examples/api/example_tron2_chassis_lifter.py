"""
@file example_tron2_chassis_lifter.py

@brief Standalone example for the Tron2 chassis / lifter state subscriptions.

Subscribes read-only feedback channels via the low-level SDK:
    - "/chassis_state" (std_msgs/Float32MultiArray) -> datatypes.ChassisState (raw data[])
    - "/lifter/state"  (controller_msgs/JointState)  -> datatypes.LifterState (q/v/tau)

Usage:
    python3 example_tron2_chassis_lifter.py [robot_ip]

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

    _last_chassis_ns = [0]
    _last_lifter_ns = [0]

    def on_chassis_state(state: datatypes.ChassisState):
        # 1 Hz throttle to avoid spamming stdout.
        if state.stamp - _last_chassis_ns[0] < 1_000_000_000:
            return
        _last_chassis_ns[0] = state.stamp
        print("[chassis-state] stamp={} data={}".format(state.stamp, list(state.data)))

    def on_lifter_state(state: datatypes.LifterState):
        if state.stamp - _last_lifter_ns[0] < 1_000_000_000:
            return
        _last_lifter_ns[0] = state.stamp
        print("[lifter-state] stamp={} q={} v={} tau={}".format(
            state.stamp, list(state.q), list(state.v), list(state.tau)))

    robot.subscribeChassisState(on_chassis_state)
    robot.subscribeLifterState(on_lifter_state)

    print("Subscribed /chassis_state and /lifter/state. Ctrl+C to exit.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
