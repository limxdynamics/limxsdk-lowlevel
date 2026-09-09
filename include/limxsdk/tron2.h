/**
 * @file tron2.h
 *
 * @brief This file contains the declarations of classes related to the control of tron2 robots.
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_TRON2_H_
#define _LIMX_SDK_TRON2_H_

#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"
#include "mros/hand_msgs/HandCmd.h"
#include "mros/hand_msgs/HandState.h"

namespace limxsdk
{
  /**
   * @brief Class for controlling a tron2 robot using the LIMX SDK API.
   */
  class LIMX_SDK_API Tron2 : public ApiBase
  {
  public:
    /**
     * @brief Get an instance of the Tron2 class.
     * @return A pointer to a Tron2 instance (Singleton pattern).
     */
    static Tron2 *getInstance();

    /**
     * @brief Pure virtual initialization method.
     *        This method should specify the operations to be performed before using the object in the main function.
     * @param robot_ip_address The IP address of the robot.
     *                         For simulation, it is typically set to "127.0.0.1",
     *                         while for a real robot, it may be set to "10.192.1.2".
     * @return True if init successfully, otherwise false.
     */
    bool init(const std::string &robot_ip_address = "127.0.0.1") override;

    /**
     * @brief Get the number of motors in the robot.
     * @return The total number of motors.
     */
    uint32_t getMotorNumber() override;

    /**
     * @brief Override to obtain names of all robot motors.
     * Used for motor identification.
     * @return Vector of motor names; empty if unavailable.
     */
    std::vector<std::string> getMotorNames() override;

    /**
     * @brief Method to subscribe to updates of the robot's IMU (Inertial Measurement Unit) data.
     * @param cb The callback function to be invoked when new IMU data is received.
     */
    void subscribeImuData(std::function<void(const ImuDataConstPtr &)> cb) override;

    /**
     * @brief Subscribe to receive updates about the robot state.
     *
     * @param cb The callback function to be invoked when a robot state update is received.
     */
    void subscribeRobotState(std::function<void(const RobotStateConstPtr &)> cb) override;

    /**
     * @brief Method for subscribing to robot command (RobotCmd) updates
     * @details Register a callback function that will be invoked when updated robot command (RobotCmd) data is received;
     *          Typical use case: Real-time collection, parsing and analysis of RobotCmd data during physical robot operation
     * @param cb Callback function for command updates, which takes a constant smart pointer to RobotCmd (RobotCmdConstPtr)
     *           as the input parameter and has no return value
     */
    void subscribeRobotCmd(std::function<void(const RobotCmdConstPtr &)> cb) override;

    /**
     * @brief Publish a command to control the robot's actions.
     *
     * @param cmd The RobotCmd object representing the desired robot command.
     * @return True if the command was successfully published, otherwise false.
     */
    bool publishRobotCmd(const RobotCmd &cmd) override;

    /**
     * @brief Method to subscribe to sensor inputs related to a joystick from the robot.
     * @param cb The callback function to be invoked when sensor input from a joystick is received from the robot.
     */
    void subscribeSensorJoy(std::function<void(const SensorJoyConstPtr &)> cb) override;

    /**
     * @brief Method to subscribe to diagnostic values from the robot.
     *
     * Examples:
     * | name        | level  | code | msg
     * |-------------|--------|------|--------------------
     * | imu         | OK     | 0    | - IMU is functioning properly.
     * | imu         | ERROR  | -1   | - Error in IMU.
     * |-------------|--------|------|--------------------
     * | ethercat    | OK     | 0    | - EtherCAT is working fine.
     * | ethercat    | ERROR  | -1   | - EtherCAT error.
     * |-------------|--------|------|--------------------
     * | calibration | OK     | 0    | - Robot calibration successful.
     * | calibration | WARN   | 1    | - Robot calibration in progress.
     * | calibration | ERROR  | -1   | - Robot calibration failed.
     * |-------------|--------|------|--------------------
     *
     * @param cb The callback function to be invoked when diagnostic values are received from the robot.
     */
    void subscribeDiagnosticValue(std::function<void(const DiagnosticValueConstPtr &)> cb) override;

    /**
     * @brief Publish a command to drive the Tron2 2-finger gripper.
     *
     * The command is published on the dedicated topic "/limx/2F-gripper/cmd"
     * (controller_msgs/JointCmd, na=2) with index layout [left, right].
     *
     * Each entry is a normalized percentage in the closed range [0, 100]:
     *   - cmd.opening[0/1] : commanded opening of the left / right finger
     *   - cmd.speed[0/1]   : commanded motion speed
     *   - cmd.force[0/1]   : commanded grasping force
     *
     * To keep parity with the existing signaling / web API, this call internally clamps:
     *   - all values to [0, 100]
     *   - cmd.opening[1] (right finger) to [0, 95]
     * Override the upper-right limit only if you are certain the firmware accepts it.
     *
     * @param cmd  GripperCmd with size-2 opening/speed/force vectors.
     * @return     true if the command was successfully published.
     */
    bool publishGripperCmd(const GripperCmd &cmd);

    /**
     * @brief Subscribe to feedback from the Tron2 2-finger gripper.
     *
     * The state is fed from "/limx/2F-gripper/state" (controller_msgs/JointState, na=2),
     * dispatched to the callback by a dedicated background thread.
     *
     * @param cb  Callback invoked when a new GripperState arrives.
     */
    void subscribeGripperState(std::function<void(const GripperStateConstPtr &)> cb);

    /**
     * @brief Subscribe to the commands being sent to the Tron2 2-finger gripper.
     *
     * Read-back channel for "/limx/2F-gripper/cmd" (controller_msgs/JointCmd, na=2).
     * Mirrors subscribeRobotCmd(): it lets an application observe what is being
     * commanded, including commands issued by other nodes, without taking control.
     *
     * @param cb  Callback invoked when a new GripperCmd is observed.
     */
    void subscribeGripperCmd(std::function<void(const GripperCmdConstPtr &)> cb);

    /**
     * @brief Subscribe to lifting-column feedback.
     *
     * Fed from "/lifter/state" (controller_msgs/JointState), published by the
     * lifting-column controller. Only the mobile dual-arm configuration has a lifter;
     * on other configurations no frame ever arrives and the callback is never invoked.
     *
     * @param cb  Callback invoked when a new LifterState arrives.
     */
    void subscribeLifterState(std::function<void(const LifterStateConstPtr &)> cb);

    /**
     * @brief Subscribe to lifting-column status in millimetres, with fault flags.
     *
     * Fed from "/lifter/status" (std_msgs/Float32MultiArray) at ~20 Hz. This is the
     * channel to use alongside publishLifterVel() / publishLifterPos(): its
     * position and velocity are already in the same millimetre units as those
     * commands, and its flags are the only way to see that a command was rejected
     * (uncalibrated, no feedback yet) or altered (speed fallback, target clamped).
     *
     * Unlike subscribeLifterState(), frames arrive whenever the chassis node is up,
     * including before any lifter command has been sent. See LifterStatus for the
     * flag list and for the level-versus-event distinction.
     *
     * @param cb  Callback invoked when a new LifterStatus arrives.
     */
    void subscribeLifterStatus(std::function<void(const LifterStatusConstPtr &)> cb);

    /**
     * @brief Subscribe to wheeled-base motion feedback.
     *
     * Fed from "/chassis_state" (std_msgs/Float32MultiArray) at ~30 Hz. See
     * ChassisState for the field layout and for why the timestamp is a receive time.
     *
     * @param cb  Callback invoked when a new ChassisState arrives.
     */
    void subscribeChassisState(std::function<void(const ChassisStateConstPtr &)> cb);

    /**
     * @brief Subscribe to the measured end-effector pose of both arms.
     *
     * Fed from "/arm_pose" (std_msgs/Float32MultiArray, 14 floats). See ArmEePose for
     * the index layout, the quaternion order and the reference-frame caveat.
     *
     * @param cb  Callback invoked when a new ArmEePose arrives.
     */
    void subscribeArmEePose(std::function<void(const ArmEePoseConstPtr &)> cb);

    /**
     * @brief Subscribe to dexterous-hand feedback.
     *
     * Fed from "/brainco2/hand/state" (hand_msgs/HandState) for the BrainCo
     * second-generation hand pair.
     *
     * @param cb  Callback invoked when a new DexHandState arrives.
     */
    void subscribeDexHandState(std::function<void(const DexHandStateConstPtr &)> cb);

    /**
     * @brief Publish a command to the dexterous hand pair.
     *
     * Published on "/brainco2/hand/cmd" (hand_msgs/HandCmd). @c cmd.ctrl_mode and
     * @c cmd.hands must both have size 2, ordered [left, right]. Per hand, the finger
     * vectors that matter depend on the control mode:
     *   1 = position + time  -> pos, time
     *   2 = position + speed -> pos, vel
     *   3 = current          -> current
     * A mode of 0 stops that hand and ignores the finger vectors.
     *
     * Finger vectors are zero-padded / truncated to the hand's finger count (6).
     * Note that padding is with 0, so in mode 1 a short @c time vector yields a
     * zero motion time on the unset fingers; always fill all 6 entries there.
     *
     * @param cmd  The dexterous-hand command to publish.
     * @return     true if the command was successfully published.
     */
    bool publishDexHandCmd(const DexHandCmd &cmd);

    /**
     * @brief Subscribe to VR head-set / hand-controller state.
     *
     * Fed from "/vr_cmd" (teleop_msgs/VRState). This is an observation channel only;
     * see VrState for details.
     *
     * @param cb  Callback invoked when a new VrState arrives.
     */
    void subscribeVrState(std::function<void(const VrStateConstPtr &)> cb);

    /**
     * @brief Drive the wheeled mobile base.
     *
     * Published on "/sdk_cmd_vel" (geometry_msgs/Twist), which is the base's
     * dedicated SDK input.
     *
     * @warning This is a STREAMING interface, not a one-shot command. ranger_node
     * stops the base if no frame arrives for 300 ms, so keep publishing at about
     * 10 Hz for as long as motion should continue, and send a zero frame to stop.
     *
     * Arguments are NORMALISED to [-1, 1], not physical units: the base scales
     * them by its own configured @c v_scale / @c sc_scale. Values outside the
     * range are clamped.
     *
     * The Twist field @c linear.z is reserved by the base for lifting-column jog,
     * and a non-zero @c linear.z suppresses base motion for that frame. To keep
     * that from surprising callers, this method always sends @c linear.z = 0, so
     * base motion is never suppressed. Use publishLifterVel() / publishLifterPos()
     * for the lifting column; they run on separate topics and therefore work
     * while the base is moving.
     *
     * @param linear_x   Forward / backward, normalised [-1, 1].
     * @param angular_z  Steering, normalised [-1, 1].
     * @param linear_y   Lateral; accepted but currently ignored by the Ackermann base.
     * @return           true if the command was successfully published.
     */
    bool publishChassisTwist(float linear_x, float angular_z, float linear_y = 0.0f);

    /**
     * @brief Drive the lifting column by velocity.
     *
     * Published on "/sdk_lifter_vel" (controller_msgs/JointCmd) as @c v[0].
     *
     * @warning STREAMING interface: publish at about 10 Hz while motion should
     * continue. If frames stop for more than 300 ms the column latches its
     * measured position and holds.
     *
     * @c vel_mm_s == 0 means "stop now": the column latches its measured position
     * and releases the streaming control slot. That stop frame is accepted even
     * without position feedback.
     *
     * Base behaviour to be aware of: a speed magnitude outside
     * (0, @c lifter_max_vel_mm_s] falls back to the configured default speed
     * rather than being clamped to the limit; travel limits are enforced by the
     * base; and frames are dropped outright while the column is uncalibrated or
     * "/lifter/state" feedback is missing, so nothing ever moves blind.
     *
     * Do not publish to "/lifter/cmd": that is ranger_node's own output to the
     * controller, and writing it fights the node.
     *
     * @param vel_mm_s  Velocity in mm/s; positive raises, negative lowers, 0 stops.
     * @return          true if the command was successfully published.
     */
    bool publishLifterVel(float vel_mm_s);

    /**
     * @brief Drive the lifting column to a height.
     *
     * Published on "/sdk_lifter_pos" (controller_msgs/JointCmd) as @c q[0] with
     * @c v[0] as the speed limit.
     *
     * @warning STREAMING interface, same 300 ms watchdog as publishLifterVel().
     * Keep publishing while the move should continue; the target may be changed
     * on any frame. On arrival the column holds rather than ending the task.
     *
     * There is no "0 = stop" convention here, because 0 mm is the valid lowest
     * position. To stop, send one publishLifterVel(0) frame or simply stop
     * publishing and let the watchdog latch.
     *
     * Targets outside the column's travel are clamped by the base. The same
     * uncalibrated / no-feedback drop rules as publishLifterVel() apply.
     *
     * @param pos_mm            Target height in mm, 0 = lowest position.
     * @param vel_limit_mm_s    Speed limit in mm/s; absolute value is used. Pass 0
     *                          to let the base apply its configured default speed.
     * @return                  true if the command was successfully published.
     */
    bool publishLifterPos(float pos_mm, float vel_limit_mm_s = 0.0f);

    /**
     * @brief Subscribe to tele-operation status.
     *
     * Decoded from the "TeleOperation" entry of "/diagnostics_value". See
     * TeleopState for the exact value convention; in particular the take-over
     * flag is edge published, so @c takeover_known is false until the first
     * take-over frame arrives.
     *
     * @param cb  Callback invoked when a "TeleOperation" diagnostic arrives.
     */
    void subscribeTeleopState(std::function<void(const TeleopStateConstPtr &)> cb);

    /**
     * @brief Subscribe to one named entry of the robot diagnostic stream.
     *
     * Filters "/diagnostics_value" by @p name and delivers every matching frame
     * as-is, including @c frame_id. This is the general-purpose escape hatch for
     * the diagnostics that have no dedicated SDK accessor; run
     * @c mrostopic echo /diagnostics_value on the robot to discover names.
     *
     * Unlike the older subscribeDiagnosticValue(), frames are neither coalesced
     * per name nor delayed at start-up, and @c frame_id is preserved.
     *
     * @param name  Exact diagnostic name to match, e.g. "TeleOperation".
     * @param cb    Callback invoked for each matching frame.
     */
    void subscribeDiagnosticByName(const std::string &name,
                                   std::function<void(const DiagnosticEntryConstPtr &)> cb);

    /**
     * @brief Fetch the robot's URDF description.
     *
     * The robot does not publish its URDF on any topic; it is a file served by
     * the rbtmgr HTTP endpoint. This helper performs that HTTP exchange with a
     * plain socket, so it adds no third-party dependency to the SDK.
     *
     * @warning BLOCKING call, not a subscription. It performs a few short HTTP
     * requests against the robot reached by init(), so call it during set-up
     * rather than from a control loop.
     *
     * The URDF file name is derived from the robot serial number the same way
     * rbtmgr derives it, and the candidates are tried in the same order, so a
     * TRON2B falls back to the matching TRON2A resource. This mirrors logic that
     * lives in rbtmgr, so if the on-robot layout changes this may need updating;
     * a dedicated read endpoint on rbtmgr would remove the duplication.
     *
     * @param urdf        Receives the URDF XML on success; untouched on failure.
     * @param port        rbtmgr HTTP port on the robot.
     * @param timeout_ms  Per-request send/receive timeout in milliseconds.
     * @return            true if a non-empty URDF was retrieved.
     */
    bool getRobotDescription(std::string &urdf, int port = 5000, int timeout_ms = 3000);

    /**
     * @brief Subscribe to lifter (vertical lift) state feedback.
     *
     * The state is fed from "/lifter/state" (controller_msgs/JointState) and
     * exposed via the existing RobotState DTO (q/dq/tau/motor_names), dispatched
     * by a dedicated background thread.
     *
     * @param cb  Callback invoked when a new lifter RobotState arrives.
     */
    void subscribeLifterState(std::function<void(const RobotStateConstPtr &)> cb);

    /**
     * @brief Publish brainco dexterous hand command to "/brainco2/hand/cmd".
     * @param cmd hand_msgs::HandCmd payload.
     * @return true if the command was successfully published.
     */
    bool publishBraincoHandCmd(const mros::hand_msgs::HandCmd &cmd);

    /**
     * @brief Subscribe to brainco dexterous hand command topic "/brainco2/hand/cmd".
     * @param cb Callback invoked when a new HandCmd arrives.
     */
    void subscribeBraincoHandCmd(std::function<void(const mros::hand_msgs::HandCmdConstPtr &)> cb);

    /**
     * @brief Subscribe to brainco dexterous hand state topic "/brainco2/hand/state".
     * @param cb Callback invoked when a new HandState arrives.
     */
    void subscribeBraincoHandState(std::function<void(const mros::hand_msgs::HandStateConstPtr &)> cb);

    /**
     * @brief Set the robot light effect.
     *
     * This method configures the robot's light effect based on the provided effect parameter.
     * The effect parameter should be an integer corresponding to one of the values defined in the
     * `LightEffect` enum, which specifies the desired robot light effect.
     *
     * The `LightEffect` enum provides various options for the light effect, including static colors,
     * flashing colors with different intensities, and fast or slow flashing patterns.
     *
     * Example usage:
     * @code
     * robot.setRobotLightEffect(Tron2::LightEffect::STATIC_RED);  // Sets the robot's light to a static red color
     * robot.setRobotLightEffect(Tron2::LightEffect::FAST_FLASH_BLUE);  // Sets the robot's light to fast-flashing blue
     * @endcode
     *
     * @param effect An integer representing the desired robot light effect, as defined in the `Tron2::LightEffect` enum.
     * @return A boolean value indicating whether the robot light effect was successfully set.
     */
    bool setRobotLightEffect(int effect) override;

    // Enum type that defines different robot light effects
    enum LightEffect : int
    {
      STATIC_RED = 0,    // Static red light
      STATIC_GREEN,      // Static green light
      STATIC_BLUE,       // Static blue light
      STATIC_CYAN,       // Static cyan light
      STATIC_PURPLE,     // Static purple light
      STATIC_YELLOW,     // Static yellow light
      STATIC_WHITE,      // Static white light
      LOW_FLASH_RED,     // Low-intensity flashing red light (slow bursts)
      LOW_FLASH_GREEN,   // Low-intensity flashing green light (slow bursts)
      LOW_FLASH_BLUE,    // Low-intensity flashing blue light (slow bursts)
      LOW_FLASH_CYAN,    // Low-intensity flashing cyan light (slow bursts)
      LOW_FLASH_PURPLE,  // Low-intensity flashing purple light (slow bursts)
      LOW_FLASH_YELLOW,  // Low-intensity flashing yellow light (slow bursts)
      LOW_FLASH_WHITE,   // Low-intensity flashing white light (slow bursts)
      FAST_FLASH_RED,    // Fast flashing red light (quick bursts)
      FAST_FLASH_GREEN,  // Fast flashing green light (quick bursts)
      FAST_FLASH_BLUE,   // Fast flashing blue light (quick bursts)
      FAST_FLASH_CYAN,   // Fast flashing cyan light (quick bursts)
      FAST_FLASH_PURPLE, // Fast flashing purple light (quick bursts)
      FAST_FLASH_YELLOW, // Fast flashing yellow light (quick bursts)
      FAST_FLASH_WHITE   // Fast flashing white light (quick bursts)
    };

    /**
     * @brief Destructor for the Tron2 class.
     *        Cleans up any resources used by the object.
     */
    virtual ~Tron2();

  private:
    /**
     * @brief Private constructor to prevent external instantiation of the Tron2 class.
     */
    Tron2();

    // Callbacks registered via subscribeGripperState(); invoked from the dedicated
    // gripper dispatch thread started inside Tron2::init().
    std::vector<std::function<void(const GripperStateConstPtr &)>> gripper_state_callback_;

    // Callbacks registered via the lifter RobotState / brainco hand subscriptions;
    // invoked from their dedicated dispatch threads started inside Tron2::init().
    std::vector<std::function<void(const RobotStateConstPtr &)>> lifter_state_callback_;
    std::vector<std::function<void(const mros::hand_msgs::HandCmdConstPtr &)>> brainco_hand_cmd_callback_;
    std::vector<std::function<void(const mros::hand_msgs::HandStateConstPtr &)>> brainco_hand_state_callback_;

    // The channels added for the lifter status / chassis / arm-pose / dexterous-hand
    // / VR interfaces keep all of their state in file-static storage inside
    // tron2.cpp rather than as members here.
  };
}

#endif
