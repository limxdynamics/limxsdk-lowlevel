/**
 * @file datatypes.h
 *
 * @brief This file contains the declarations of classes and structures related to robotics.
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_DATATYPES_H_
#define _LIMX_SDK_DATATYPES_H_

#include <stdint.h>
#include <vector>
#include <memory>
#include <string>

namespace limxsdk
{
  /**
   * @struct ImuData
   *
   * @brief Structure representing IMU (Inertial Measurement Unit) data of a robot based on sensor feedback.
   *
   * This structure encapsulates IMU data including accelerometer, gyroscope, and quaternion.
   */
  struct ImuData
  {
    ImuData()
    {
      for (int i = 0; i < 3; i++)
      {
        acc[i] = 0.0;
      }
      for (int i = 0; i < 3; i++)
      {
        gyro[i] = 0.0;
      }
      for (int i = 0; i < 4; i++)
      {
        quat[i] = 0.0;
      }
    }
    uint64_t stamp; // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    float acc[3];   // Array to store IMU accelerometer data for tracking linear acceleration along three axes (X, Y, Z).
    float gyro[3];  // Array to store IMU gyroscope data for tracking angular velocity or rotational speed along three axes (X, Y, Z).
    float quat[4];  // Array to store IMU quaternion data, representing orientation in 3D space (w, x, y, z).
  };
  typedef std::shared_ptr<ImuData> ImuDataPtr;
  typedef std::shared_ptr<ImuData const> ImuDataConstPtr;

  /**
   * @struct RobotState
   *
   * @brief Structure representing the state of a robot based on sensor feedback.
   *
   * This structure encapsulates various data points that could be used to monitor and control a robot, including output torque, current angles, and velocities.
   */
  struct RobotState
  {
    RobotState() {}
    RobotState(int motor_num)
        : tau(motor_num, 0.0), q(motor_num, 0.0), dq(motor_num, 0.0), motor_names(motor_num, "") {}

    void resize(int motor_num)
    {
      tau.resize(motor_num, 0.0);
      q.resize(motor_num, 0.0);
      dq.resize(motor_num, 0.0);
      motor_names.resize(motor_num, "");
    }

    uint64_t stamp;                       // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    std::vector<float> tau;               // Vector to store the current estimated output torque (in Newton meters)
    std::vector<float> q;                 // Vector to store the current angles (in radians)
    std::vector<float> dq;                // Vector to store the current velocities (in radians per second)
    std::vector<std::string> motor_names; // Vector storing the names of the motors
  };
  typedef std::shared_ptr<RobotState> RobotStatePtr;
  typedef std::shared_ptr<RobotState const> RobotStateConstPtr;

  /**
   * @struct RobotCmd
   *
   * @brief Structure representing the command for controlling a robot.
   *
   * This structure holds various commands that can be used to control a robot, including the desired working mode, desired angles, desired velocities, desired output torque, desired position stiffness, and desired velocity stiffness.
   */
  struct RobotCmd
  {
    RobotCmd() {}
    RobotCmd(int motor_num)
        : mode(motor_num, 0)
        , q(motor_num, 0.0)
        , dq(motor_num, 0.0)
        , tau(motor_num, 0.0)
        , Kp(motor_num, 0.0)
        , Kd(motor_num, 0.0)
        , motor_names(motor_num, "")
        , parallel_solve_required(motor_num, true) {}

    void resize(int motor_num)
    {
      mode.resize(motor_num, 0);
      q.resize(motor_num, 0.0);
      dq.resize(motor_num, 0.0);
      tau.resize(motor_num, 0.0);
      Kp.resize(motor_num, 0.0);
      Kd.resize(motor_num, 0.0);
      motor_names.resize(motor_num, "");
      parallel_solve_required.resize(motor_num, true);
    }

    uint64_t stamp;                       // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    std::vector<uint8_t> mode;            // The desired working mode of each motor. The control modes are defined as follows:
                                          //   0: Torque-position hybrid control mode
                                          //   1: Velocity control mode
                                          //   2: Position control mode
                                          //   3: Torque control mode
    std::vector<float> q;                 // Vector storing the desired angles (in radians).
    std::vector<float> dq;                // Vector storing the desired velocities (in radians per second).
    std::vector<float> tau;               // Vector storing the desired output torque (in Newton meters).
    std::vector<float> Kp;                // Vector storing the desired position stiffness (in Newton meters per radian).
    std::vector<float> Kd;                // Vector storing the desired velocity stiffness (in Newton meters per radian per second).
    std::vector<std::string> motor_names; // Vector storing the names of the motors
    std::vector<bool> parallel_solve_required; // Flag indicating whether parallel solving is required for each motor
  };
  typedef std::shared_ptr<RobotCmd> RobotCmdPtr;
  typedef std::shared_ptr<RobotCmd const> RobotCmdConstPtr;

  /**
   * @struct SensorJoy
   *
   * @brief Structure representing sensor inputs from robot joystick.
   *
   * This structure contains timestamp information along with axis and button values obtained from a joystick sensor.
   */
  struct SensorJoy
  {
    uint64_t stamp;               // Timestamp in nanoseconds, associated with the sensor input.
    std::vector<float> axes;      // Values representing the positions of the joystick axes.
    std::vector<int32_t> buttons; // Values representing the state of joystick buttons.
  };
  typedef std::shared_ptr<SensorJoy> SensorJoyPtr;
  typedef std::shared_ptr<SensorJoy const> SensorJoyConstPtr;

  /**
   * @struct DiagnosticValue
   *
   * @brief Structure representing diagnostic values.
   *
   * This structure contains information about the diagnostic level, name, code, and message.
   */
  struct DiagnosticValue
  {
#if defined(_WIN32) && defined(OK)
#undef OK
#endif
#if defined(_WIN32) && defined(WARN)
#undef WARN
#endif
#if defined(_WIN32) && defined(ERROR)
#undef ERROR
#endif
    enum { OK = 0 };
    enum { WARN = 1 };
    enum { ERROR = 2 };

    uint64_t stamp;      // Timestamp in nanoseconds.
    int32_t level;       // Level associated with the diagnostic value.
    std::string name;    // Name identifying the diagnostic value.
    int32_t code;        // Code corresponding to the diagnostic value.
    std::string message; // Detailed message related to the diagnostic value.
  };
  typedef std::shared_ptr<DiagnosticValue> DiagnosticValuePtr;
  typedef std::shared_ptr<DiagnosticValue const> DiagnosticValueConstPtr;

  /**
   * @struct GripperCmd
   *
   * @brief Structure representing a command to drive a multi-finger gripper.
   *
   * Each vector is indexed per finger. For Tron2 the layout is [left, right] (size=2).
   * Values are normalized percentages in the range [0, 100]:
   *   - opening : commanded opening (0 = fully closed, 100 = fully open)
   *   - speed   : commanded motion speed
   *   - force   : commanded grasping force
   *
   * The interface is intentionally generic (variable-length vectors) so that 2F / 3F / 5F
   * grippers can share the same data structure.
   */
  struct GripperCmd
  {
    GripperCmd() {}
    explicit GripperCmd(int finger_num)
        : opening(finger_num, 0.0f), speed(finger_num, 0.0f), force(finger_num, 0.0f) {}

    void resize(int finger_num)
    {
      opening.resize(finger_num, 0.0f);
      speed.resize(finger_num, 0.0f);
      force.resize(finger_num, 0.0f);
    }

    uint64_t stamp{0};            // Timestamp in nanoseconds.
    std::vector<float> opening;   // Commanded opening per finger, in [0, 100] (%).
    std::vector<float> speed;     // Commanded motion speed per finger, in [0, 100] (%).
    std::vector<float> force;     // Commanded grasping force per finger, in [0, 100] (%).
  };
  typedef std::shared_ptr<GripperCmd> GripperCmdPtr;
  typedef std::shared_ptr<GripperCmd const> GripperCmdConstPtr;

  /**
   * @struct GripperState
   *
   * @brief Structure representing the feedback state of a multi-finger gripper.
   *
   * Index layout follows the command (e.g. Tron2: [left, right]).
   */
  struct GripperState
  {
    GripperState() {}
    explicit GripperState(int finger_num)
        : q(finger_num, 0.0f), v(finger_num, 0.0f), vd(finger_num, 0.0f), tau(finger_num, 0.0f) {}

    uint64_t stamp{0};        // Timestamp in nanoseconds.
    std::vector<float> q;     // Current opening feedback per finger (%).
    std::vector<float> v;     // Current motion speed feedback per finger.
    std::vector<float> vd;    // Desired/raw velocity feedback (controller-defined).
    std::vector<float> tau;   // Current force/torque feedback per finger.
  };
  typedef std::shared_ptr<GripperState> GripperStatePtr;
  typedef std::shared_ptr<GripperState const> GripperStateConstPtr;

  /**
   * @struct TerrainData
   *
   * @brief Structure representing terrain data collected by the robot's sensors.
   *
   * This structure contains timestamp information along with serialized terrain-related data 
   * (e.g., elevation values, slope parameters, or terrain point cloud coordinates) 
   * in a double-precision vector format.
   */
  struct TerrainData
  {
    uint64_t stamp{0};               // Timestamp in nanoseconds
    uint32_t height{0};              // Vertical dimension (rows) of the terrain data grid (in pixels/points)
    uint32_t width{0};               // Horizontal dimension (columns) of the terrain data grid (in pixels/points)
    uint32_t step{1};                // Number of bytes/elements between consecutive rows in the data vector (stride)
    std::vector<double> data;        // Serialized terrain data (e.g., elevation values, slope angles, 3D XYZ coordinates)
                                     // Data layout: row-major (row-by-row) order matching height/width dimensions
  };
  typedef std::shared_ptr<TerrainData> TerrainDataPtr;
  typedef std::shared_ptr<TerrainData const> TerrainDataConstPtr;

  /**
   * @struct LifterState
   *
   * @brief Structure representing the feedback state of the lifting column ("lifter").
   *
   * Fed from the "/lifter/state" topic (controller_msgs/JointState) published by the
   * lifting-column controller. Only present on the mobile dual-arm configuration.
   *
   * @note The values are raw controller/motor-side quantities, NOT millimetres. The
   *       conversion to millimetres of travel is a per-machine calibration
   *       (position_mm = (q[0] - q_at_zero_mm) / q_per_mm) that lives in the
   *       ranger_node configuration, not in this SDK.
   */
  struct LifterState
  {
    uint64_t stamp{0};                     // Timestamp in nanoseconds (from the message header).
    uint32_t na{0};                        // Number of lifter joints reported by the controller.
    std::vector<std::string> joint_names;  // Joint names as reported by the controller.
    std::vector<float> q;                  // Current position feedback (controller units).
    std::vector<float> v;                  // Current velocity feedback (controller units per second).
    std::vector<float> vd;                 // Desired/raw velocity feedback (controller-defined).
    std::vector<float> tau;                // Current torque/force feedback.
  };
  typedef std::shared_ptr<LifterState> LifterStatePtr;
  typedef std::shared_ptr<LifterState const> LifterStateConstPtr;

  /**
   * @struct LifterStatus
   *
   * @brief Lifting-column status in physical units, with fault flags.
   *
   * Fed from the "/lifter/status" topic (std_msgs/Float32MultiArray, published at
   * ~20 Hz by the chassis node whenever it is running, including before any lifter
   * command has ever been sent). The wire layout is a fixed-length float array:
   *   data[0] = position in mm
   *   data[1] = velocity in mm/s
   *   data[2] = flags bitmask
   *   data[3] = control owner (internal, for diagnostics)
   *   data[4] = active task (internal, for diagnostics)
   *
   * Prefer this over LifterState for anything user-facing: the chassis node has
   * already applied the per-machine calibration, so @c position_mm and
   * @c velocity_mm_s are in the same millimetre units as the arguments of
   * Tron2::publishLifterPos() / Tron2::publishLifterVel(). LifterState remains
   * available for the raw controller-side quantities.
   *
   * The flags are the only way to observe why a lifter command had no effect: a
   * command sent while NOT_CALIBRATED or STATE_NOT_READY is set is dropped by the
   * chassis node, and one whose speed was out of range comes back as
   * SPEED_FALLBACK rather than being clamped.
   *
   * @note std_msgs/Float32MultiArray carries no header, so @c stamp is the time at
   *       which this SDK received the frame, not a publisher timestamp.
   * @note @c valid is false if the frame was shorter than expected, in which case
   *       only @c data has been populated.
   */
  struct LifterStatus
  {
    /**
     * @brief Bit masks for the @c flags field.
     *
     * LEVEL flags are recomputed every frame and can be read directly as "the
     * current condition". EVENT flags are one-shot pulses: they are raised for a
     * single published frame and then cleared, so an application that cares about
     * them has to latch them itself (a second or so is a reasonable hold time).
     */
    enum Flag
    {
      WATCHDOG_LATCHED = 0x001,  // LEVEL: streaming input timed out; measured position latched and held.
      AT_LIMIT_LOWER   = 0x002,  // LEVEL: at the bottom of travel. Normal condition, not a fault.
      AT_LIMIT_UPPER   = 0x004,  // LEVEL: at the top of travel. Normal condition, not a fault.
      LAG_SATURATED    = 0x008,  // EVENT: command-vs-measured lag hit its limit; mechanism may be stuck.
      SPEED_FALLBACK   = 0x010,  // EVENT: requested speed was out of range, configured default used instead.
      CMD_CLAMPED      = 0x020,  // EVENT: target position was clamped to the travel range.
      NOT_CALIBRATED   = 0x040,  // LEVEL: column not calibrated; every lifter command is rejected.
      STATE_NOT_READY  = 0x080,  // LEVEL: no "/lifter/state" feedback yet; commands rejected to avoid blind motion.
      FRAME_IGNORED    = 0x100,  // EVENT: a streaming frame was dropped because a higher-priority owner held the column.
      PREEMPTED        = 0x200   // EVENT: the running task was preempted by a higher-priority command.
    };

    uint64_t stamp{0};             // SDK receive time in nanoseconds (no publisher stamp available).
    float position_mm{0.0f};       // data[0]: current height in mm, 0 = lowest position.
    float velocity_mm_s{0.0f};     // data[1]: current velocity in mm/s, positive = rising.
    uint32_t flags{0};             // data[2]: bitmask of Flag values.
    int32_t owner{0};              // data[3]: which input owns the column (internal diagnostics).
    int32_t task{0};               // data[4]: which task is running (internal diagnostics).
    bool valid{false};             // False if the frame was too short to decode.
    std::vector<float> data;       // Raw Float32MultiArray payload as received.
  };
  typedef std::shared_ptr<LifterStatus> LifterStatusPtr;
  typedef std::shared_ptr<LifterStatus const> LifterStatusConstPtr;

  /**
   * @struct ChassisState
   *
   * @brief Structure representing the motion state of the wheeled mobile base.
   *
   * Fed from the "/chassis_state" topic (std_msgs/Float32MultiArray, published at
   * ~30 Hz by the chassis node). The wire layout is a flat float array:
   *   data[0] = linear velocity
   *   data[1] = angular velocity
   *   data[2] = steering angle
   *
   * The three named fields are decoded copies of data[0..2]; @c data keeps the raw
   * array so that a future firmware that appends more entries stays readable without
   * an SDK change.
   *
   * @note std_msgs/Float32MultiArray carries no header, so @c stamp is the time at
   *       which this SDK received the frame, not a publisher timestamp.
   * @note The chassis node stops publishing while the chassis link is down, so a
   *       stale @c stamp means "no fresh chassis data", not "zero velocity".
   */
  struct ChassisState
  {
    uint64_t stamp{0};              // SDK receive time in nanoseconds (no publisher stamp available).
    float linear_velocity{0.0f};    // data[0]: linear velocity of the base.
    float angular_velocity{0.0f};   // data[1]: angular velocity of the base.
    float steering_angle{0.0f};     // data[2]: current steering angle.
    std::vector<float> data;        // Raw Float32MultiArray payload as received.
  };
  typedef std::shared_ptr<ChassisState> ChassisStatePtr;
  typedef std::shared_ptr<ChassisState const> ChassisStateConstPtr;

  /**
   * @struct ArmEePose
   *
   * @brief Structure representing the measured end-effector pose of both arms.
   *
   * Fed from the "/arm_pose" topic (std_msgs/Float32MultiArray). The wire layout is a
   * flat array of 14 floats, 7 per arm, ordered [x, y, z, qw, qx, qy, qz]:
   *   data[0..2]   left  position
   *   data[3..6]   left  orientation quaternion (w, x, y, z)
   *   data[7..9]   right position
   *   data[10..13] right orientation quaternion (w, x, y, z)
   *
   * @note The quaternion order is w-first, matching the manipulation controller's
   *       Cartesian servo protocol (xyz + qwxyz).
   * @note std_msgs/Float32MultiArray carries no header, so @c stamp is the SDK
   *       receive time.
   * @note The reference frame of these poses is defined by the manipulation
   *       controller and is not declared on the wire. Treat the values as
   *       controller-frame and confirm the frame with the controller team before
   *       using them for absolute positioning.
   */
  struct ArmEePose
  {
    ArmEePose()
    {
      for (int i = 0; i < 3; i++)
      {
        left_position[i] = 0.0f;
        right_position[i] = 0.0f;
      }
      for (int i = 0; i < 4; i++)
      {
        left_quat[i] = 0.0f;
        right_quat[i] = 0.0f;
      }
    }

    uint64_t stamp{0};         // SDK receive time in nanoseconds (no publisher stamp available).
    bool valid{false};         // True when the frame carried the full 14-float layout.
    float left_position[3];    // Left end-effector position (x, y, z).
    float left_quat[4];        // Left end-effector orientation (w, x, y, z).
    float right_position[3];   // Right end-effector position (x, y, z).
    float right_quat[4];       // Right end-effector orientation (w, x, y, z).
    std::vector<float> data;   // Raw Float32MultiArray payload as received.
  };
  typedef std::shared_ptr<ArmEePose> ArmEePosePtr;
  typedef std::shared_ptr<ArmEePose const> ArmEePoseConstPtr;

  /**
   * @struct DexHandFingers
   *
   * @brief Per-hand finger payload of a dexterous hand command or state.
   *
   * All vectors are indexed per finger and share the same length (6 for the
   * supported BrainCo second-generation hand).
   */
  struct DexHandFingers
  {
    std::vector<std::string> finger_names; // Finger names as reported by the hand.
    std::vector<float> pos;                // Finger positions.
    std::vector<float> vel;                // Finger velocities.
    std::vector<float> current;            // Finger currents (used by force control).
    std::vector<float> time;               // Per-finger motion time (used by position-time control).
  };

  /**
   * @struct DexHandCmd
   *
   * @brief Structure representing a command to a pair of dexterous hands.
   *
   * Published on "/brainco2/hand/cmd" (hand_msgs/HandCmd). Index layout for
   * @c ctrl_mode and @c hands is [left, right] (size 2, both always present).
   *
   * @c ctrl_mode selects which of the finger vectors is actually honoured:
   *   0 = stop
   *   1 = position + time   (fill @c pos and @c time)
   *   2 = position + speed  (fill @c pos and @c vel)
   *   3 = current / force   (fill @c current)
   *
   * This is the SDK-side counterpart of the "hand mode" and "hand time" quantities:
   * they are fields of this command, not independent topics.
   */
  struct DexHandCmd
  {
    DexHandCmd() : ctrl_mode(2, 0), hands(2) {}

    uint64_t stamp{0};                  // Timestamp in nanoseconds.
    std::string hand_type;              // Optional hand model identifier; empty = controller default.
    std::vector<uint8_t> ctrl_mode;     // Control mode per hand, size 2: [left, right].
    std::vector<DexHandFingers> hands;  // Finger payload per hand, size 2: [left, right].
  };
  typedef std::shared_ptr<DexHandCmd> DexHandCmdPtr;
  typedef std::shared_ptr<DexHandCmd const> DexHandCmdConstPtr;

  /**
   * @struct DexHandState
   *
   * @brief Structure representing the feedback state of a pair of dexterous hands.
   *
   * Fed from "/brainco2/hand/state" (hand_msgs/HandState). Index layout matches the
   * command: [left, right].
   */
  struct DexHandState
  {
    DexHandState() : ctrl_mode(2, 0), hands(2) {}

    uint64_t stamp{0};                  // Timestamp in nanoseconds (from the message header).
    std::string hand_type;              // Hand model identifier reported by the hand.
    std::vector<uint8_t> ctrl_mode;     // Active control mode per hand, size 2: [left, right].
    std::vector<DexHandFingers> hands;  // Finger feedback per hand, size 2: [left, right].
  };
  typedef std::shared_ptr<DexHandState> DexHandStatePtr;
  typedef std::shared_ptr<DexHandState const> DexHandStateConstPtr;

  /**
   * @struct VrState
   *
   * @brief Structure representing one frame of VR head-set / hand-controller state.
   *
   * Fed from the "/vr_cmd" topic (teleop_msgs/VRState) published by the VR bridge.
   * This is a read-only observation channel: it lets an application watch what the
   * physical VR device is reporting. Injecting synthetic VR input is a separate
   * channel ("/sdk_vr_cmd") and is intentionally not exposed here.
   *
   * The three pose members are row-major 4x4 homogeneous transforms (16 floats).
   */
  struct VrState
  {
    VrState()
    {
      for (int i = 0; i < 16; i++)
      {
        eye_pose[i] = 0.0f;
        left_pose[i] = 0.0f;
        right_pose[i] = 0.0f;
      }
      for (int i = 0; i < 2; i++)
      {
        left_joystick[i] = 0.0f;
        right_joystick[i] = 0.0f;
      }
    }

    uint64_t stamp{0};          // Timestamp in nanoseconds (from the message header).
    float eye_pose[16];         // Head-set pose, row-major 4x4 transform.
    float left_pose[16];        // Left controller pose, row-major 4x4 transform.
    float right_pose[16];       // Right controller pose, row-major 4x4 transform.

    float left_joystick[2];     // Left thumb-stick axes.
    float left_trigger{0.0f};   // Left trigger travel.
    float left_grip{0.0f};      // Left grip travel.
    bool left_thumb_touch{false};   // Left thumb-stick touched.
    bool left_trigger_pressed{false};
    bool left_grip_pressed{false};
    bool button_x{false};       // Left-hand button X.
    bool button_y{false};       // Left-hand button Y.

    float right_joystick[2];    // Right thumb-stick axes.
    float right_trigger{0.0f};  // Right trigger travel.
    float right_grip{0.0f};     // Right grip travel.
    bool right_thumb_touch{false};  // Right thumb-stick touched.
    bool right_trigger_pressed{false};
    bool right_grip_pressed{false};
    bool button_a{false};       // Right-hand button A.
    bool button_b{false};       // Right-hand button B.
  };
  typedef std::shared_ptr<VrState> VrStatePtr;
  typedef std::shared_ptr<VrState const> VrStateConstPtr;

  /**
   * @struct DiagnosticEntry
   *
   * @brief One raw entry of the robot diagnostic stream, including the frame id.
   *
   * This mirrors mros diagnostic_msgs/DiagnosticValue one-to-one and is delivered
   * per frame without coalescing. It exists alongside the older DiagnosticValue
   * because that structure cannot carry @c frame_id, which several diagnostics
   * (notably "TeleOperation") use to distinguish sub-meanings of the same name.
   */
  struct DiagnosticEntry
  {
    enum { OK = 0, WARN = 1, ERR = 2 };

    uint64_t stamp{0};      // Timestamp in nanoseconds (from the message header).
    std::string name;       // Diagnostic item name, e.g. "TeleOperation".
    std::string frame_id;   // Header frame id; qualifies the meaning of some items.
    int32_t level{0};       // Severity: 0 = OK, 1 = WARN, 2 = ERROR.
    int32_t code{0};        // Item-specific numeric payload.
    std::string message;    // Item-specific text payload.
  };
  typedef std::shared_ptr<DiagnosticEntry> DiagnosticEntryPtr;
  typedef std::shared_ptr<DiagnosticEntry const> DiagnosticEntryConstPtr;

  /**
   * @struct TeleopState
   *
   * @brief Decoded view of the "TeleOperation" diagnostic.
   *
   * The robot publishes two distinct pieces of information under the single
   * diagnostic name "TeleOperation", told apart by the frame id:
   *
   *   - frame_id == "VRTakeOver" and level == OK
   *       @c code carries a VR take-over flag: 1 = take-over engaged,
   *       0 = released. This is what @c takeover_active reflects.
   *   - any frame with level == ERROR
   *       A tele-operation fault, with the detail in @c message. This is what
   *       @c healthy reflects.
   *
   * Because the take-over flag is edge published (only sent when it changes),
   * @c takeover_known stays false until the first "VRTakeOver" frame is seen.
   * Treat "not known" as "unknown", not as "not in tele-operation".
   */
  struct TeleopState
  {
    uint64_t stamp{0};             // Timestamp in nanoseconds (from the message header).
    bool takeover_known{false};    // True once a "VRTakeOver" frame has been observed.
    bool takeover_active{false};   // True while VR tele-operation take-over is engaged.
    bool healthy{true};            // False when the last frame reported level == ERROR.
    int32_t level{0};              // Raw diagnostic level of the frame that produced this.
    int32_t code{0};               // Raw diagnostic code.
    std::string frame_id;          // Raw frame id ("VRTakeOver" for take-over frames).
    std::string message;           // Raw message; carries the fault detail when unhealthy.
  };
  typedef std::shared_ptr<TeleopState> TeleopStatePtr;
  typedef std::shared_ptr<TeleopState const> TeleopStateConstPtr;
}

#endif