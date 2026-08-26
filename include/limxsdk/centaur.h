/**
 * @file centaur.h
 *
 * @brief Low-level SDK API for a composed centaur robot.
 *
 * The lower body uses the default MROS topics. The independent upper body uses
 * the per-endpoint DID topic prefix "/did_upbody".
 */

#ifndef _LIMX_SDK_CENTAUR_H_
#define _LIMX_SDK_CENTAUR_H_

#include "limxsdk/apibase.h"
#include "limxsdk/macros.h"

namespace limxsdk
{
  class LIMX_SDK_API Centaur : public ApiBase
  {
  public:
    static Centaur *getInstance();

    bool init(const std::string &robot_ip_address = "127.0.0.1") override;

    // The inherited, unqualified API is kept as a lower-body alias.
    uint32_t getMotorNumber() override;
    std::vector<std::string> getMotorNames() override;
    void subscribeImuData(std::function<void(const ImuDataConstPtr &)> cb) override;
    void subscribeRobotState(std::function<void(const RobotStateConstPtr &)> cb) override;
    void subscribeRobotCmd(std::function<void(const RobotCmdConstPtr &)> cb) override;
    bool publishRobotCmd(const RobotCmd &cmd) override;

    uint32_t getLowerBodyMotorNumber();
    std::vector<std::string> getLowerBodyMotorNames();
    void subscribeLowerBodyImuData(std::function<void(const ImuDataConstPtr &)> cb);
    void subscribeLowerBodyRobotState(std::function<void(const RobotStateConstPtr &)> cb);
    void subscribeLowerBodyRobotCmd(std::function<void(const RobotCmdConstPtr &)> cb);
    bool publishLowerBodyRobotCmd(const RobotCmd &cmd);

    uint32_t getUpperBodyMotorNumber();
    std::vector<std::string> getUpperBodyMotorNames();
    void subscribeUpperBodyImuData(std::function<void(const ImuDataConstPtr &)> cb);
    void subscribeUpperBodyRobotState(std::function<void(const RobotStateConstPtr &)> cb);
    void subscribeUpperBodyRobotCmd(std::function<void(const RobotCmdConstPtr &)> cb);
    bool publishUpperBodyRobotCmd(const RobotCmd &cmd);

    virtual ~Centaur();

  private:
    Centaur();
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };
}

#endif
