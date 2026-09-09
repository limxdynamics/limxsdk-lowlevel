"""
@file example_tron2_machine_data.py

@brief Standalone example for the Tron2 machine-data read-back APIs.

Subscribes to every machine-data channel the low-level SDK exposes and prints a
throttled summary of each one:

    Robot.subscribeGripperCmd()    -> "/limx/2F-gripper/cmd"  (command read-back)
    Robot.subscribeLifterState()   -> "/lifter/state"         (raw controller units)
    Robot.subscribeLifterStatus()  -> "/lifter/status"        (mm + fault flags)
    Robot.subscribeChassisState()  -> "/chassis_state"         (mobile dual-arm only)
    Robot.subscribeArmEePose()     -> "/arm_pose"
    Robot.subscribeDexHandState()  -> "/brainco2/hand/state"
    Robot.subscribeVrState()       -> "/vr_cmd"
    Robot.subscribeTeleopState()   -> "/diagnostics_value" ("TeleOperation")
    Robot.subscribeDiagnosticByName() -> "/diagnostics_value" (--diag=<name>)

Channels whose data source is absent on the current machine simply stay silent;
the example reports which ones never produced a frame when you stop it.

The write channels are opt-in, because they move real hardware:

    --send-hand-cmd    one open-hand command on "/brainco2/hand/cmd"
    --drive-chassis    5 s of slow forward motion on "/sdk_cmd_vel" (10 Hz)
    --drive-lifter     5 s of slow raise then a stop frame on "/sdk_lifter_vel"
    --lifter-pos=<mm>  5 s of position streaming on "/sdk_lifter_pos"

Chassis and lifter inputs are STREAMING: the base stops on its own roughly
300 ms after the frames stop, which is why the loops below keep publishing
rather than sending one frame.

    --urdf             blocking HTTP fetch of the URDF, prints its size

Usage:
    python3 example_tron2_machine_data.py [robot_ip] [options]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import sys
import time

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes

PRINT_PERIOD_S = 2.0

# Per-channel bookkeeping: frame count plus the last time we printed, so a 500 Hz
# topic cannot flood stdout.
_stats = {}


def _should_print(channel: str) -> bool:
    entry = _stats.setdefault(channel, {"count": 0, "last": 0.0})
    entry["count"] += 1
    now = time.monotonic()
    if now - entry["last"] < PRINT_PERIOD_S:
        return False
    entry["last"] = now
    return True


def _fmt(values, n=4, digits=3):
    """Format the first n entries of a float sequence, marking truncation."""
    head = ["{:.{}f}".format(v, digits) for v in list(values)[:n]]
    suffix = ", ..." if len(values) > n else ""
    return "[" + ", ".join(head) + suffix + "]"


STREAM_HZ = 10
STREAM_SECONDS = 5


def _stream(publish, label):
    """Drive a streaming write channel for STREAM_SECONDS at STREAM_HZ.

    Stopping the loop is what stops the motion, so this is the command.
    """
    print("[{}] streaming for {} s at {} Hz".format(label, STREAM_SECONDS, STREAM_HZ))
    for _ in range(STREAM_HZ * STREAM_SECONDS):
        if not publish():
            print("[{}] publish rejected".format(label))
            return
        time.sleep(1.0 / STREAM_HZ)


def _opt_value(prefix, default=None):
    for arg in sys.argv[1:]:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    send_hand_cmd = "--send-hand-cmd" in sys.argv
    drive_chassis = "--drive-chassis" in sys.argv
    drive_lifter = "--drive-lifter" in sys.argv
    fetch_urdf = "--urdf" in sys.argv
    lifter_pos = _opt_value("--lifter-pos=")
    diag_name = _opt_value("--diag=")
    robot_ip = args[0] if args else "10.192.1.2"

    robot = Robot(RobotType.Tron2)
    if not robot.init(robot_ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)

    def on_gripper_cmd(cmd: datatypes.GripperCmd):
        if not _should_print("gripper_cmd"):
            return
        print("[gripper-cmd ] opening={} speed={} force={}".format(
            _fmt(cmd.opening), _fmt(cmd.speed), _fmt(cmd.force)))

    def on_lifter_state(state: datatypes.LifterState):
        if not _should_print("lifter_state"):
            return
        print("[lifter      ] na={} q={} v={} (raw controller units)".format(
            state.na, _fmt(state.q), _fmt(state.v)))

    _LIFTER_FLAG_NAMES = (
        ("NOT_CALIBRATED", datatypes.LifterStatus.NOT_CALIBRATED),
        ("STATE_NOT_READY", datatypes.LifterStatus.STATE_NOT_READY),
        ("WATCHDOG_LATCHED", datatypes.LifterStatus.WATCHDOG_LATCHED),
        ("SPEED_FALLBACK", datatypes.LifterStatus.SPEED_FALLBACK),
        ("CMD_CLAMPED", datatypes.LifterStatus.CMD_CLAMPED),
        ("FRAME_IGNORED", datatypes.LifterStatus.FRAME_IGNORED),
    )

    def on_lifter_status(status: datatypes.LifterStatus):
        if not _should_print("lifter_status"):
            return
        if not status.valid:
            print("[lifter-stat ] unexpected payload length {}".format(len(status.data)))
            return
        # Spell out the flags that explain why a lifter command would be refused
        # or altered, since that is the whole point of reading this channel.
        notes = [name for name, mask in _LIFTER_FLAG_NAMES if status.flags & mask]
        print("[lifter-stat ] pos={:.1f}mm vel={:.1f}mm/s flags=0x{:03x} {}".format(
            status.position_mm, status.velocity_mm_s, status.flags,
            " ".join(notes)))

    def on_chassis_state(state: datatypes.ChassisState):
        if not _should_print("chassis_state"):
            return
        print("[chassis     ] linear={:.3f} angular={:.3f} steering={:.3f}".format(
            state.linear_velocity, state.angular_velocity, state.steering_angle))

    def on_arm_ee_pose(pose: datatypes.ArmEePose):
        if not _should_print("arm_ee_pose"):
            return
        if not pose.valid:
            print("[arm-ee-pose ] unexpected payload length {}; raw={}".format(
                len(pose.data), _fmt(pose.data, n=6)))
            return
        print("[arm-ee-pose ] L pos={} quat(wxyz)={} | R pos={} quat(wxyz)={}".format(
            _fmt(pose.left_position, n=3), _fmt(pose.left_quat),
            _fmt(pose.right_position, n=3), _fmt(pose.right_quat)))

    def on_dex_hand_state(state: datatypes.DexHandState):
        if not _should_print("dex_hand_state"):
            return
        print("[dex-hand    ] type='{}' mode={} L pos={} R pos={}".format(
            state.hand_type, list(state.ctrl_mode),
            _fmt(state.hands[0].pos, n=6), _fmt(state.hands[1].pos, n=6)))

    def on_vr_state(vr: datatypes.VrState):
        if not _should_print("vr_state"):
            return
        print("[vr          ] LJS={} trig={:.2f} grip={:.2f} X={} Y={} | "
              "RJS={} trig={:.2f} grip={:.2f} A={} B={}".format(
                  _fmt(vr.left_joystick, n=2, digits=2), vr.left_trigger,
                  vr.left_grip, vr.button_x, vr.button_y,
                  _fmt(vr.right_joystick, n=2, digits=2), vr.right_trigger,
                  vr.right_grip, vr.button_a, vr.button_b))

    def on_teleop_state(state: datatypes.TeleopState):
        # Not throttled: these frames are edge published and each one matters.
        _stats.setdefault("teleop_state", {"count": 0, "last": 0.0})["count"] += 1
        if state.takeover_known:
            takeover = "yes" if state.takeover_active else "no"
        else:
            takeover = "unknown"
        print("[teleop      ] takeover={} healthy={} level={} code={} "
              "frame='{}' msg='{}'".format(takeover, state.healthy, state.level,
                                           state.code, state.frame_id, state.message))

    def on_diagnostic(entry: datatypes.DiagnosticEntry):
        _stats.setdefault("diagnostic", {"count": 0, "last": 0.0})["count"] += 1
        print("[diag        ] name='{}' frame='{}' level={} code={} msg='{}'".format(
            entry.name, entry.frame_id, entry.level, entry.code, entry.message))

    robot.subscribeGripperCmd(on_gripper_cmd)
    robot.subscribeLifterState(on_lifter_state)
    robot.subscribeLifterStatus(on_lifter_status)
    robot.subscribeChassisState(on_chassis_state)
    robot.subscribeArmEePose(on_arm_ee_pose)
    robot.subscribeDexHandState(on_dex_hand_state)
    robot.subscribeVrState(on_vr_state)
    robot.subscribeTeleopState(on_teleop_state)

    if diag_name:
        robot.subscribeDiagnosticByName(diag_name, on_diagnostic)
        print("Filtering /diagnostics_value for name='{}'.".format(diag_name))

    if fetch_urdf:
        urdf = robot.getRobotDescription()
        if urdf:
            print("[urdf        ] fetched {} bytes, starts with: {:.60}".format(
                len(urdf), urdf))
        else:
            print("[urdf        ] fetch failed")

    if send_hand_cmd:
        cmd = datatypes.DexHandCmd()
        # Mode 2 = position + speed, so pos and vel are the fields that matter.
        cmd.ctrl_mode = [2, 2]
        for hand in cmd.hands:
            hand.pos = [0.0] * 6
            hand.vel = [10.0] * 6
        cmd.stamp = time.time_ns()
        print("[dex-hand-cmd] sending open-hand command (mode=2)")
        robot.publishDexHandCmd(cmd)

    if drive_chassis:
        _stream(lambda: robot.publishChassisTwist(0.2, 0.0), "chassis-cmd")
        # Explicit zero frame instead of relying on the watchdog.
        robot.publishChassisTwist(0.0, 0.0)
        print("[chassis-cmd ] sent stop frame")

    if drive_lifter:
        _stream(lambda: robot.publishLifterVel(20.0), "lifter-cmd")
        # v=0 latches the measured position and releases the streaming slot.
        robot.publishLifterVel(0.0)
        print("[lifter-cmd  ] sent v=0 stop frame")

    if lifter_pos is not None:
        target = float(lifter_pos)
        _stream(lambda: robot.publishLifterPos(target, 30.0), "lifter-cmd")
        # Position streaming has no "0 = stop", so stop through the velocity topic.
        robot.publishLifterVel(0.0)
        print("[lifter-cmd  ] stopped via publishLifterVel(0)")

    print("Listening for machine data; press Ctrl-C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n--- frames received per channel ---")
        for channel in ("gripper_cmd", "lifter_state", "lifter_status",
                        "chassis_state", "arm_ee_pose", "dex_hand_state",
                        "vr_state", "teleop_state", "diagnostic"):
            count = _stats.get(channel, {}).get("count", 0)
            note = "" if count else "  (no data source on this machine?)"
            print("  {:<16} {}{}".format(channel, count, note))


if __name__ == "__main__":
    main()
