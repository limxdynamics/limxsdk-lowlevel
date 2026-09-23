#include "limxsdk/ability/base_ability.h"
#include <chrono>

namespace examples {

// Turning ability
class TurnAbility : public limxsdk::ability::BaseAbility {
public:
    bool on_init(const YAML::Node& config) override {
        std::cout << "TurnAbility initialized with config: " << config << std::endl;
        
        return true;
    }
    
    void on_start() override {
        std::cout << "TurnAbility started" << std::endl;
    }
    
    void on_stop() override {
        std::cout << "TurnAbility stopped" << std::endl;
    }
    
    void on_main() override {
        std::cout << "TurnAbility running" << std::endl;
        while(isRunning())
          std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1 * 1e9)));
    }
};

// Sensor monitoring ability
class SensorAbility : public limxsdk::ability::BaseAbility {
public:
    bool on_init(const YAML::Node& config) override {
        std::cout << "SensorAbility initialized with config: " << config << std::endl;
        
        return true;
    }
    
    void on_start() override {
        std::cout << "SensorAbility started" << std::endl;
    }
    
    void on_stop() override {
        std::cout << "SensorAbility stopped" << std::endl;
    }
    
    void on_main() override {
        std::cout << "SensorAbility running" << std::endl;
        while(isRunning())
          std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1 * 1e9)));
    }
};

// Battery monitoring ability
class BatteryAbility : public limxsdk::ability::BaseAbility {
public:
    bool on_init(const YAML::Node& config) override {
        std::cout << "SensorAbility initialized with config: " << config << std::endl;
        
        return true;
    }
    
    void on_start() override {
        std::cout << "SensorAbility started" << std::endl;
    }
    
    void on_stop() override {
        std::cout << "SensorAbility stopped" << std::endl;
    }
    
    void on_main() override {
        std::cout << "SensorAbility running" << std::endl;
        while(isRunning())
          std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1 * 1e9)));
    }
};

} // namespace examples

// Register abilities
LIMX_REGISTER_ABILITY(examples::TurnAbility)
LIMX_REGISTER_ABILITY(examples::SensorAbility)
LIMX_REGISTER_ABILITY(examples::BatteryAbility)
