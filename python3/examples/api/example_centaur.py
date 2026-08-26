"""Receive and command the composed Centaur upper and lower bodies."""

import sys
import time

import limxsdk.datatypes as datatypes
from limxsdk.robot.Robot import Robot
from limxsdk.robot.RobotType import RobotType


def print_state(body):
    def callback(state):
        print(f"{body} state: names={state.motor_names}, q={state.q}")
    return callback


def print_imu(body):
    def callback(imu):
        print(f"{body} imu: acc={imu.acc}, gyro={imu.gyro}, quat={imu.quat}")
    return callback


def print_cmd(body):
    def callback(cmd):
        print(f"{body} cmd: stamp={cmd.stamp}, names={cmd.motor_names}, q={cmd.q}")
    return callback


def hold_position(robot, upper):
    if upper:
        names = robot.getUpperBodyMotorNames()
    else:
        names = robot.getLowerBodyMotorNames()

    cmd = datatypes.RobotCmd()
    cmd.stamp = time.time_ns()
    cmd.mode = [0] * len(names)
    cmd.q = [0.0] * len(names)
    cmd.dq = [0.0] * len(names)
    cmd.tau = [0.0] * len(names)
    cmd.Kp = [0.0] * len(names)
    cmd.Kd = [0.0] * len(names)
    cmd.parallel_solve_required = [False] * len(names)
    cmd.motor_names = names
    if upper:
        return robot.publishUpperBodyRobotCmd(cmd)
    return robot.publishLowerBodyRobotCmd(cmd)


if __name__ == "__main__":
    robot = Robot(RobotType.Centaur)
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "10.192.1.2"
    if not robot.init(robot_ip):
        sys.exit(1)

    robot.subscribeLowerBodyRobotState(print_state("lower"))
    robot.subscribeLowerBodyImuData(print_imu("lower"))
    robot.subscribeLowerBodyRobotCmd(print_cmd("lower"))
    robot.subscribeUpperBodyRobotState(print_state("upper"))
    robot.subscribeUpperBodyImuData(print_imu("upper"))
    robot.subscribeUpperBodyRobotCmd(print_cmd("upper"))

    while True:
        hold_position(robot, upper=False)
        hold_position(robot, upper=True)
        time.sleep(0.01)
