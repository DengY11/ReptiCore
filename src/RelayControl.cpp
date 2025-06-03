/*
 * 继电器控制模块实现
 * RelayControl.cpp
 * 现代C++17风格重构版本
 */

#include "RelayControl.h"

#include <algorithm>

namespace ReptileController::Relay {

// 全局继电器控制器实例
Controller g_controller;

void Controller::initialize() noexcept {
  if (initialized_) return;
  turnOffAll();
  initialized_ = true;
}

void Controller::setPhysicalState(Type type, State state) const noexcept {
  if (!isValidType(type)) return;

  const auto& config = configs_[static_cast<size_t>(type)];
  GPIO_PinState pinState =
      (state == State::ON)
          ? (config.activeLevel ? GPIO_PIN_SET : GPIO_PIN_RESET)
          : (config.activeLevel ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(config.port, config.pin, pinState);
}

void Controller::setState(Type type, State state) noexcept {
  if (!initialized_ || !isValidType(type)) return;

  const size_t index = static_cast<size_t>(type);
  relayStates_[index] = (state == State::ON);
  setPhysicalState(type, state);
}

State Controller::getState(Type type) const noexcept {
  if (!isValidType(type)) return State::OFF;

  const size_t index = static_cast<size_t>(type);
  return relayStates_[index] ? State::ON : State::OFF;
}

void Controller::toggleState(Type type) noexcept {
  if (!initialized_ || !isValidType(type)) return;

  const State currentState = getState(type);
  const State newState = (currentState == State::ON) ? State::OFF : State::ON;
  setState(type, newState);
}

void Controller::turnOffAll() noexcept {
  for (size_t i = 0; i < RELAY_COUNT; ++i) {
    const Type type = static_cast<Type>(i);
    setPhysicalState(type, State::OFF);
    relayStates_[i] = false;
  }
}

void Controller::safetyCheck() noexcept {
  static uint32_t lastCheckTime = 0;
  const uint32_t currentTime = HAL_GetTick();

  if (currentTime - lastCheckTime >= 1000) {
    lastCheckTime = currentTime;
    static uint32_t heaterOnTime = 0;

    if (getState(Type::HEATER) == State::ON) {
      heaterOnTime += 1000;
      // 如果加热器连续运行超过30分钟，强制关闭
      if (heaterOnTime > 30 * 60 * 1000) {
        setState(Type::HEATER, State::OFF);
        setState(Type::FAN, State::ON);  // 开启风扇散热
        heaterOnTime = 0;
      }
    } else {
      heaterOnTime = 0;  // 重置计时器
    }
  }
}

void Controller::emergencyStop() noexcept {
  turnOffAll();
  initialized_ = false;
}

}  // namespace ReptileController::Relay

// C接口实现
extern "C" {

void RelayControl_Init(void) {
  ReptileController::Relay::g_controller.initialize();
}

void RelayControl_Set(RelayType_t relay, RelayState_t state) {
  const auto type = static_cast<ReptileController::Relay::Type>(relay);
  const auto relayState = static_cast<ReptileController::Relay::State>(state);
  ReptileController::Relay::g_controller.setState(type, relayState);
}

RelayState_t RelayControl_Get(RelayType_t relay) {
  const auto type = static_cast<ReptileController::Relay::Type>(relay);
  const auto state = ReptileController::Relay::g_controller.getState(type);
  return static_cast<RelayState_t>(state);
}

void RelayControl_Toggle(RelayType_t relay) {
  const auto type = static_cast<ReptileController::Relay::Type>(relay);
  ReptileController::Relay::g_controller.toggleState(type);
}

void RelayControl_AllOff(void) {
  ReptileController::Relay::g_controller.turnOffAll();
}

RelayStatus_t RelayControl_GetStatus(void) {
  RelayStatus_t status = {};

  status.heaterState = RelayControl_Get(RELAY_HEATER);
  status.fanState = RelayControl_Get(RELAY_FAN);
  status.humidifierState = RelayControl_Get(RELAY_HUMIDIFIER);
  status.lastUpdateTime = HAL_GetTick();

  return status;
}

void RelayControl_SafetyCheck(void) {
  ReptileController::Relay::g_controller.safetyCheck();
}

void RelayControl_EmergencyStop(void) {
  ReptileController::Relay::g_controller.emergencyStop();
}

}  // extern "C"