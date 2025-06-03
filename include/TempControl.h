/*
 * 温湿度智能控制算法头文件
 * TempControl.h
 * 现代C++风格重构版本
 */

#pragma once

#include "RelayControl.h"
#include "main.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

// 自定义clamp函数（C++14兼容）
namespace detail {
template <typename T>
constexpr const T& clamp(const T& value, const T& min, const T& max) {
  return (value < min) ? min : (value > max) ? max : value;
}
}  // namespace detail

namespace ReptileController::Control {

// 控制模式枚举（使用enum class）
enum class Mode : uint8_t {
  AUTO = 0,  // 自动模式
  MANUAL,    // 手动模式
  OFF        // 关闭模式
};

// PID控制器模板类
template <typename T = float>
class PIDController {
 private:
  T kp_{0};             // 比例系数
  T ki_{0};             // 积分系数
  T kd_{0};             // 微分系数
  T integral_{0};       // 积分累积值
  T lastError_{0};      // 上次误差值
  T maxIntegral_{100};  // 积分限幅
  T outputMax_{100};    // 输出最大值
  T outputMin_{-100};   // 输出最小值

 public:
  // 构造函数使用统一初始化
  constexpr PIDController(T kp = 0, T ki = 0, T kd = 0) noexcept
      : kp_{kp}, ki_{ki}, kd_{kd} {}

  // 设置PID参数
  void setParameters(T kp, T ki, T kd) noexcept {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  // 设置输出限制
  void setOutputLimits(T min, T max) noexcept {
    outputMin_ = min;
    outputMax_ = max;
  }

  // 设置积分限幅
  void setIntegralLimit(T limit) noexcept { maxIntegral_ = limit; }

  // 更新PID控制器（移除nodiscard以兼容C++14）
  T update(T setpoint, T measurement, T deltaTime) noexcept {
    const T error = setpoint - measurement;

    // 比例项
    const T proportional = kp_ * error;

    // 积分项
    integral_ += error * deltaTime;
    integral_ = detail::clamp(integral_, -maxIntegral_, maxIntegral_);
    const T integralTerm = ki_ * integral_;

    // 微分项
    const T derivative = (deltaTime > 0) ? (error - lastError_) / deltaTime : 0;
    const T derivativeTerm = kd_ * derivative;

    // 计算输出
    const T output = proportional + integralTerm + derivativeTerm;
    lastError_ = error;

    return detail::clamp(output, outputMin_, outputMax_);
  }

  // 重置PID控制器
  void reset() noexcept {
    integral_ = 0;
    lastError_ = 0;
  }

  // Getter方法
  constexpr T getKp() const noexcept { return kp_; }
  constexpr T getKi() const noexcept { return ki_; }
  constexpr T getKd() const noexcept { return kd_; }
  constexpr T getIntegral() const noexcept { return integral_; }
  constexpr T getLastError() const noexcept { return lastError_; }
};

// 控制状态类
class State {
 private:
  Mode mode_{Mode::AUTO};
  bool heaterEnabled_{false};
  bool fanEnabled_{false};
  bool humidifierEnabled_{false};
  uint32_t lastControlTime_{0};
  float tempOutput_{0.0f};
  float humidityOutput_{0.0f};

 public:
  State() = default;

  // Getter方法（const正确性）
  Mode getMode() const noexcept { return mode_; }
  bool isHeaterEnabled() const noexcept { return heaterEnabled_; }
  bool isFanEnabled() const noexcept { return fanEnabled_; }
  bool isHumidifierEnabled() const noexcept {
    return humidifierEnabled_;
  }
  uint32_t getLastControlTime() const noexcept {
    return lastControlTime_;
  }
  float getTempOutput() const noexcept { return tempOutput_; }
  float getHumidityOutput() const noexcept {
    return humidityOutput_;
  }

  // Setter方法
  void setMode(Mode mode) noexcept { mode_ = mode; }
  void setHeaterEnabled(bool enabled) noexcept { heaterEnabled_ = enabled; }
  void setFanEnabled(bool enabled) noexcept { fanEnabled_ = enabled; }
  void setHumidifierEnabled(bool enabled) noexcept {
    humidifierEnabled_ = enabled;
  }
  void setLastControlTime(uint32_t time) noexcept { lastControlTime_ = time; }
  void setTempOutput(float output) noexcept { tempOutput_ = output; }
  void setHumidityOutput(float output) noexcept { humidityOutput_ = output; }
};

// 温湿度控制器类
class TemperatureController {
 private:
  std::unique_ptr<PIDController<float>> tempPID_;
  std::unique_ptr<PIDController<float>> humidityPID_;
  std::unique_ptr<State> state_;
  Relay::Controller* relayController_;  // 改为普通指针，指向全局实例

  // 控制算法私有方法
  void executeTemperatureControl(float currentTemp, float targetTemp,
                                 float tolerance) noexcept;
  void executeHumidityControl(float currentHumidity, float targetHumidity,
                              float tolerance) noexcept;
  void executeAutoMode(const ReptileController::SensorData& sensorData,
                       const ReptileController::ControlConfig& config) noexcept;
  void executeManualMode() noexcept;

 public:
  // 构造函数
  TemperatureController();

  // 析构函数
  ~TemperatureController() = default;

  // 禁用拷贝构造和赋值
  TemperatureController(const TemperatureController&) = delete;
  TemperatureController& operator=(const TemperatureController&) = delete;

  // 允许移动构造和赋值
  TemperatureController(TemperatureController&&) = default;
  TemperatureController& operator=(TemperatureController&&) = default;

  // 初始化控制器
  void initialize() noexcept;

  // 主更新函数
  void update(const ReptileController::SensorData& sensorData,
              const ReptileController::ControlConfig& config) noexcept;

  // 模式控制
  void setMode(Mode mode) noexcept;
  Mode getMode() const noexcept;

  // 获取控制状态
  const State& getState() const noexcept { return *state_; }

  // PID参数调整
  void setTemperaturePIDParameters(float kp, float ki, float kd) noexcept;
  void setHumidityPIDParameters(float kp, float ki, float kd) noexcept;

  // 目标值设置
  void setTargetTemperature(float temperature) noexcept;
  void setTargetHumidity(float humidity) noexcept;
  void setTolerances(float tempTolerance, float humidityTolerance) noexcept;

  // 紧急停止
  void emergencyStop() noexcept;

  // 安全检查
  bool safetyCheck() const noexcept;
};

// 全局控制器实例
extern std::unique_ptr<TemperatureController> g_controller;

}  // namespace ReptileController::Control

// C接口部分
#ifdef __cplusplus
extern "C" {
#endif

// 控制模式枚举（C风格）
typedef enum {
  CONTROL_MODE_AUTO = 0,
  CONTROL_MODE_MANUAL,
  CONTROL_MODE_OFF
} ControlMode_t;

// PID控制器结构体（C风格）
typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float lastError;
  float maxIntegral;
  float outputMax;
  float outputMin;
} PID_t;

// 控制状态结构体（C风格）
typedef struct {
  ControlMode_t mode;
  bool heaterEnabled;
  bool fanEnabled;
  bool humidifierEnabled;
  uint32_t lastControlTime;
  float tempOutput;
  float humidityOutput;
} ControlState_t;

// 传感器数据结构体（C风格）
typedef struct SensorData {
  float temperature;
  float humidity;
  uint32_t timestamp;
  bool valid;
} SensorData_t;

// 控制配置结构体（C风格）
typedef struct ControlConfig {
  float targetTemperature;
  float targetHumidity;
  float tempTolerance;
  float humidityTolerance;
} ControlConfig_t;

// C接口函数声明
void TempControl_Init(void);
void TempControl_Update(SensorData_t* sensorData, ControlConfig_t* config);
void TempControl_SetMode(ControlMode_t mode);
ControlMode_t TempControl_GetMode(void);
ControlState_t TempControl_GetState(void);

// PID控制器C接口
void PID_Init(PID_t* pid, float kp, float ki, float kd);
float PID_Update(PID_t* pid, float setpoint, float measurement, float dt);
void PID_Reset(PID_t* pid);

// 内部函数声明
void TempControl_TemperatureControl(float currentTemp, float targetTemp,
                                    float tolerance);
void TempControl_HumidityControl(float currentHumidity, float targetHumidity,
                                 float tolerance);
void TempControl_AutoMode(SensorData_t* sensorData, ControlConfig_t* config);
void TempControl_ManualMode(void);

// 参数设置
void TempControl_SetTargetTemperature(float temperature);
void TempControl_SetTargetHumidity(float humidity);
void TempControl_SetTolerance(float tempTolerance, float humidityTolerance);

#ifdef __cplusplus
}
#endif