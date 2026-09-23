#include <cstdlib>
#include "limxsdk/pointfoot.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

void platform_sleep(int seconds) {
#ifdef _WIN32
  Sleep(seconds * 1000);
#else
  sleep(seconds);
#endif
}

int main(int argc, char *argv[]) {
  limxsdk::PointFoot* robot = limxsdk::PointFoot::getInstance();

  std::string robot_ip = "127.0.0.1";
  if (argc > 1) {
    robot_ip = argv[1]; 
  }

  if (!robot->init(robot_ip)) {
    std::exit(1);
  }

  while(true) {
    printf("limxsdk::PointFoot::STATIC_RED\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::STATIC_RED);
    platform_sleep(5);

    printf("limxsdk::PointFoot::STATIC_GREEN\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::STATIC_GREEN);
    platform_sleep(5);

    printf("limxsdk::PointFoot::STATIC_BLUE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::STATIC_BLUE);
    platform_sleep(5);

    printf("limxsdk::PointFoot::STATIC_CYAN\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::STATIC_CYAN);
    platform_sleep(5);

    printf("limxsdk::PointFoot::STATIC_PURPLE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::STATIC_PURPLE);
    platform_sleep(5);

    printf("limxsdk::PointFoot::STATIC_YELLOW\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::STATIC_YELLOW);
    platform_sleep(5);

    printf("limxsdk::PointFoot::STATIC_WHITE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::STATIC_WHITE);
    platform_sleep(5);

    printf("limxsdk::PointFoot::LOW_FLASH_RED\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::LOW_FLASH_RED);
    platform_sleep(5);

    printf("limxsdk::PointFoot::LOW_FLASH_GREEN\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::LOW_FLASH_GREEN);
    platform_sleep(5);

    printf("limxsdk::PointFoot::LOW_FLASH_BLUE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::LOW_FLASH_BLUE);
    platform_sleep(5);

    printf("limxsdk::PointFoot::LOW_FLASH_CYAN\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::LOW_FLASH_CYAN);
    platform_sleep(5);

    printf("limxsdk::PointFoot::LOW_FLASH_PURPLE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::LOW_FLASH_PURPLE);
    platform_sleep(5);

    printf("limxsdk::PointFoot::LOW_FLASH_YELLOW\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::LOW_FLASH_YELLOW);
    platform_sleep(5);

    printf("limxsdk::PointFoot::LOW_FLASH_WHITE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::LOW_FLASH_WHITE);
    platform_sleep(5);

    printf("limxsdk::PointFoot::FAST_FLASH_RED\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::FAST_FLASH_RED);
    platform_sleep(5);

    printf("limxsdk::PointFoot::FAST_FLASH_GREEN\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::FAST_FLASH_GREEN);
    platform_sleep(5);

    printf("limxsdk::PointFoot::FAST_FLASH_BLUE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::FAST_FLASH_BLUE);
    platform_sleep(5);

    printf("limxsdk::PointFoot::FAST_FLASH_CYAN\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::FAST_FLASH_CYAN);
    platform_sleep(5);

    printf("limxsdk::PointFoot::FAST_FLASH_PURPLE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::FAST_FLASH_PURPLE);
    platform_sleep(5);

    printf("limxsdk::PointFoot::FAST_FLASH_YELLOW\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::FAST_FLASH_YELLOW);
    platform_sleep(5);

    printf("limxsdk::PointFoot::FAST_FLASH_WHITE\n");
    robot->setRobotLightEffect(limxsdk::PointFoot::FAST_FLASH_WHITE);
    platform_sleep(5);
  }

  return 0;
}    