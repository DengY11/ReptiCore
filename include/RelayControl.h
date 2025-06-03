/*
 * 继电器控制模块头文件
 * RelayControl.h
 * 现代C++风格重构版本
 */

#pragma once

#include "main.h"

#include <array>
#include <bitset>
#include <cstdint>

namespace ReptileController::Relay {

// 继电器类型枚举（使用enum class）
enum class Type : uint8_t {
  HEATER = 0,  // 加热器
  FAN,         // 风扇
  HUMIDIFIER,  // 雾化器
  COUNT        // 继电器总数
};

// 继电器状态枚举
enum class State : uint8_t {
  OFF = 0,  // 关闭
  ON = 1    // 开启
};

// 继电器配置结构
struct Config {
  GPIO_TypeDef* port;
  uint16_t pin;
  bool activeLevel;  // true = 高电平有效, false = 低电平有效

  constexpr Config(GPIO_TypeDef* p, uint16_t pin_num, bool active_high = false)
      : port(p), pin(pin_num), activeLevel(active_high) {}
};

// 继电器控制器类
class Controller {
 private:
  static constexpr size_t RELAY_COUNT = static_cast<size_t>(Type::COUNT);

  // 继电器配置数组（运行时初始化）
  std::array<Config, RELAY_COUNT> configs_;

  // 继电器状态位集
  std::bitset<RELAY_COUNT> relayStates_{};
  bool initialized_{false};

  // 私有方法（移除nodiscard以兼容C++14）
  bool isValidType(Type type) const noexcept {
    return static_cast<size_t>(type) < RELAY_COUNT;
  }

  void setPhysicalState(Type type, State state) const noexcept;

 public:
  Controller() : configs_{{{ReptileController::Pins::Relay::HEATER_PORT,
                            ReptileController::Pins::Relay::HEATER_PIN, false},
                           {ReptileController::Pins::Relay::FAN_PORT,
                            ReptileController::Pins::Relay::FAN_PIN, false},
                           {ReptileController::Pins::Relay::HUMIDIFIER_PORT,
                            ReptileController::Pins::Relay::HUMIDIFIER_PIN, false}}} {}
  
  ~Controller() = default;

  // 禁用拷贝构造和赋值（控制器是唯一的）
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  // 允许移动构造和赋值
  Controller(Controller&&) = default;
  Controller& operator=(Controller&&) = default;

  // 初始化继电器控制器
  void initialize() noexcept;

  // 设置继电器状态
  void setState(Type type, State state) noexcept;

  // 获取继电器状态
  State getState(Type type) const noexcept;

  // 切换继电器状态
  void toggleState(Type type) noexcept;

  // 关闭所有继电器
  void turnOffAll() noexcept;

  // 获取所有继电器状态
  std::bitset<RELAY_COUNT> getAllStates() const noexcept {
    return relayStates_;
  }

  // 检查是否已初始化
  bool isInitialized() const noexcept { return initialized_; }

  // 获取继电器配置
  const Config& getConfig(Type type) const noexcept {
    return configs_[static_cast<size_t>(type)];
  }

  // 安全检查（防止设备过热等异常）
  void safetyCheck() noexcept;

  // 紧急停止（立即关闭所有继电器）
  void emergencyStop() noexcept;
};

// 全局继电器控制器实例
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