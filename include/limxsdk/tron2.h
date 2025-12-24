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
     * @brief Destructor for the Tron2 class.
     *        Cleans up any resources used by the object.
     */
    virtual ~Tron2();

  private:
    /**
     * @brief Private constructor to prevent external instantiation of the Tron2 class.
     */
    Tron2();
  };
}

#endif