/*
 * 继电器控制模块头文件
 * RelayControl.h
 * 现代C++风格重构版本
 */

#pragma once

#include <array>
#include <bitset>
#include <cstdint>

#include "main.h"

namespace ReptileController::Relay {

enum class Type : uint8_t {
  HEATER = 0,  // 加热器
  FAN,         // 风扇
  HUMIDIFIER,  // 雾化器
  COUNT        // 继电器总数
};

enum class State : uint8_t { OFF = 0, ON = 1 };

struct Config {
  GPIO_TypeDef* port;
  uint16_t pin;
  bool activeLevel;
  constexpr Config(GPIO_TypeDef* p, uint16_t pin_num, bool active_high = false)
      : port(p), pin(pin_num), activeLevel(active_high) {}
};

class Controller {
 private:
  static constexpr size_t RELAY_COUNT = static_cast<size_t>(Type::COUNT);
  std::array<Config, RELAY_COUNT> configs_;
  std::bitset<RELAY_COUNT> relayStates_{};
  bool initialized_{false};
  bool isValidType(Type type) const noexcept {
    return static_cast<size_t>(type) < RELAY_COUNT;
  }
  void setPhysicalState(Type type, State state) const noexcept;

 public:
  Controller()
      : configs_{{{ReptileController::Pins::Relay::HEATER_PORT,
                   ReptileController::Pins::Relay::HEATER_PIN, false},
                  {ReptileController::Pins::Relay::FAN_PORT,
                   ReptileController::Pins::Relay::FAN_PIN, false},
                  {ReptileController::Pins::Relay::HUMIDIFIER_PORT,
                   ReptileController::Pins::Relay::HUMIDIFIER_PIN, false}}} {}

  ~Controller() = default;
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  Controller(Controller&&) = default;
  Controller& operator=(Controller&&) = default;
  void initialize() noexcept;
  void setState(Type type, State state) noexcept;
  State getState(Type type) const noexcept;
  void toggleState(Type type) noexcept;
  void turnOffAll() noexcept;
  std::bitset<RELAY_COUNT> getAllStates() const noexcept {
    return relayStates_;
  }
  bool isInitialized() const noexcept { return initialized_; }
  const Config& getConfig(Type type) const noexcept {
    return configs_[static_cast<size_t>(type)];
  }
  void safetyCheck() noexcept;
  void emergencyStop() noexcept;
};

extern Controller g_controller;

}  // namespace ReptileController::Relay

// C接口部分
#ifdef __cplusplus
extern "C" {
#endif

// C风格接口（为了兼容性）
typedef enum { RELAY_HEATER = 0, RELAY_FAN, RELAY_HUMIDIFIER } RelayType_t;

typedef enum { RELAY_OFF = 0, RELAY_ON = 1 } RelayState_t;

// 继电器状态结构体
typedef struct {
  RelayState_t heaterState;
  RelayState_t fanState;
  RelayState_t humidifierState;
  uint32_t lastUpdateTime;
} RelayStatus_t;

// C函数声明
void RelayControl_Init(void);
void RelayControl_Set(RelayType_t relay, RelayState_t state);
RelayState_t RelayControl_Get(RelayType_t relay);
void RelayControl_Toggle(RelayType_t relay);
void RelayControl_AllOff(void);
RelayStatus_t RelayControl_GetStatus(void);
void RelayControl_SafetyCheck(void);
void RelayControl_EmergencyStop(void);

#ifdef __cplusplus
}
#endif