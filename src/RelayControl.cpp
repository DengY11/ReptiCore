/*
 * 继电器控制模块实现
 * RelayControl.cpp
 * 现代C++风格重构版本
 */

#include "RelayControl.h"

#include <chrono>
#include <memory>

namespace ReptileController::Relay {

// 全局继电器控制器实例
Controller g_controller;

// 继电器控制器实现
void Controller::initialize() noexcept {
  // 初始化所有继电器为关闭状态
  turnOffAll();

  // 初始化GPIO配置
  for (size_t i = 0; i < RELAY_COUNT; ++i) {
    const auto& config = configs_[i];

    // 设置初始状态
    HAL_GPIO_WritePin(config.port, config.pin,
                      config.activeLevel ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }

  initialized_ = true;
}

void Controller::setState(Type type, State state) noexcept {
  if (!isValidType(type) || !initialized_) return;

  const size_t index = static_cast<size_t>(type);
  const bool isOn = (state == State::ON);

  // 更新状态位集
  relayStates_[index] = isOn;

  // 设置物理状态
  setPhysicalState(type, state);

  // 执行安全检查
  safetyCheck();
}

State Controller::getState(Type type) const noexcept {
  if (!isValidType(type)) return State::OFF;

  const size_t index = static_cast<size_t>(type);
  return relayStates_[index] ? State::ON : State::OFF;
}

void Controller::toggleState(Type type) noexcept {
  const State currentState = getState(type);
  const State newState = (currentState == State::ON) ? State::OFF : State::ON;
  setState(type, newState);
}

void Controller::turnOffAll() noexcept {
  for (size_t i = 0; i < RELAY_COUNT; ++i) {
    setState(static_cast<Type>(i), State::OFF);
  }
}

void Controller::setPhysicalState(Type type, State state) const noexcept {
  if (!isValidType(type)) return;

  const size_t index = static_cast<size_t>(type);
  const auto& config = configs_[index];

  // 根据activeLevel决定GPIO状态
  GPIO_PinState pinState;
  if (config.activeLevel) {
    // 高电平有效
    pinState = (state == State::ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  } else {
    // 低电平有效
    pinState = (state == State::ON) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(config.port, config.pin, pinState);
}

void Controller::safetyCheck() noexcept {
  static uint32_t lastCheck = 0;
  static uint32_t heatFanStartTime = 0;
  static bool cooldownActive = false;
  static uint32_t cooldownStartTime = 0;

  const uint32_t currentTime = HAL_GetTick();

  // 每秒检查一次
  if (currentTime - lastCheck < 1000) {
    return;
  }
  lastCheck = currentTime;

  const bool heaterOn = getState(Type::HEATER) == State::ON;
  const bool fanOn = getState(Type::FAN) == State::ON;

  // 如果加热器和风扇都开启超过30分钟，强制关闭加热器5分钟
  if (heaterOn && fanOn) {
    if (heatFanStartTime == 0) {
      heatFanStartTime = currentTime;
    } else if (currentTime - heatFanStartTime > 30 * 60 * 1000) {  // 30分钟
      // 开始冷却周期
      setState(Type::HEATER, State::OFF);
      cooldownActive = true;
      cooldownStartTime = currentTime;
      heatFanStartTime = 0;
    }
  } else {
    heatFanStartTime = 0;
  }

  // 冷却周期管理
  if (cooldownActive &&
      (currentTime - cooldownStartTime > 5 * 60 * 1000)) {  // 5分钟冷却
    cooldownActive = false;
  }

  // 如果在冷却周期内，禁止开启加热器
  if (cooldownActive && heaterOn) {
    setState(Type::HEATER, State::OFF);
  }
}

void Controller::emergencyStop() noexcept {
  // 立即关闭所有继电器的物理状态
  for (size_t i = 0; i < RELAY_COUNT; ++i) {
    const auto& config = configs_[i];
    // 强制设置为无效状态（关闭）
    HAL_GPIO_WritePin(config.port, config.pin,
                      config.activeLevel ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }

  // 重置状态
  relayStates_.reset();  // 全部设为false
}

}  // namespace ReptileController::Relay

extern "C" {

// 继电器状态记录（为C接口兼容性）
static RelayStatus_t legacyStatus = {.heaterState = RELAY_OFF,
                                     .fanState = RELAY_OFF,
                                     .humidifierState = RELAY_OFF,
                                     .lastUpdateTime = 0};

// C风格接口的辅助函数
static ReptileController::Relay::Type convertToModernType(RelayType_t legacy) {
  switch (legacy) {
    case RELAY_HEATER:
      return ReptileController::Relay::Type::HEATER;
    case RELAY_FAN:
      return ReptileController::Relay::Type::FAN;
    case RELAY_HUMIDIFIER:
      return ReptileController::Relay::Type::HUMIDIFIER;
    default:
      return ReptileController::Relay::Type::HEATER;  // 默认值
  }
}

static ReptileController::Relay::State convertToModernState(
    RelayState_t legacy) {
  return (legacy == RELAY_ON) ? ReptileController::Relay::State::ON
                              : ReptileController::Relay::State::OFF;
}

static RelayState_t convertToLegacyState(
    ReptileController::Relay::State modern) {
  return (modern == ReptileController::Relay::State::ON) ? RELAY_ON : RELAY_OFF;
}

static void updateLegacyStatus() {
  using namespace ReptileController::Relay;

  legacyStatus.heaterState =
      convertToLegacyState(g_controller.getState(Type::HEATER));
  legacyStatus.fanState =
      convertToLegacyState(g_controller.getState(Type::FAN));
  legacyStatus.humidifierState =
      convertToLegacyState(g_controller.getState(Type::HUMIDIFIER));
  legacyStatus.lastUpdateTime = HAL_GetTick();
}

// C风格接口实现
void RelayControl_Init(void) {
  ReptileController::Relay::g_controller.initialize();
  updateLegacyStatus();
}

void RelayControl_Set(RelayType_t relay, RelayState_t state) {
  const auto modernType = convertToModernType(relay);
  const auto modernState = convertToModernState(state);

  ReptileController::Relay::g_controller.setState(modernType, modernState);
  updateLegacyStatus();
}

RelayState_t RelayControl_Get(RelayType_t relay) {
  const auto modernType = convertToModernType(relay);
  const auto modernState =
      ReptileController::Relay::g_controller.getState(modernType);
  return convertToLegacyState(modernState);
}

void RelayControl_Toggle(RelayType_t relay) {
  const auto modernType = convertToModernType(relay);
  ReptileController::Relay::g_controller.toggleState(modernType);
  updateLegacyStatus();
}

void RelayControl_AllOff(void) {
  ReptileController::Relay::g_controller.turnOffAll();
  updateLegacyStatus();
}

RelayStatus_t RelayControl_GetStatus(void) {
  updateLegacyStatus();
  return legacyStatus;
}

void RelayControl_SafetyCheck(void) {
  ReptileController::Relay::g_controller.safetyCheck();
  updateLegacyStatus();
}

void RelayControl_EmergencyStop(void) {
  ReptileController::Relay::g_controller.emergencyStop();
  updateLegacyStatus();
}

}  // extern "C"