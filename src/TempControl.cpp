/*
 * 温湿度智能控制算法实现
 * TempControl.cpp
 * 现代C++风格重构版本
 */

#include "TempControl.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "RelayControl.h"

// 命名空间别名
namespace RC = ReptileController;

// 自定义clamp函数（C++14兼容）
template <typename T>
constexpr const T& clamp(const T& value, const T& min, const T& max) {
  return (value < min) ? min : (value > max) ? max : value;
}

// 全局实例声明
extern RC::SensorData g_sensorData;
extern RC::ControlConfig g_controlConfig;

namespace ReptileController::Control {

// 温控器全局实例
std::unique_ptr<TemperatureController> g_controller = nullptr;

// 温控器构造函数
TemperatureController::TemperatureController()
    : tempPID_(std::make_unique<PIDController<float>>(2.0f, 0.1f, 0.5f)),
      humidityPID_(std::make_unique<PIDController<float>>(1.5f, 0.05f, 0.3f)),
      state_(std::make_unique<State>()),
      relayController_(&Relay::g_controller) {
  // 设置PID输出限制
  tempPID_->setOutputLimits(-100.0f, 100.0f);
  humidityPID_->setOutputLimits(-100.0f, 100.0f);

  // 设置积分限幅
  tempPID_->setIntegralLimit(50.0f);
  humidityPID_->setIntegralLimit(50.0f);
}

void TemperatureController::initialize() noexcept {
  // 初始化控制状态
  state_->setMode(Mode::AUTO);
  state_->setHeaterEnabled(false);
  state_->setFanEnabled(false);
  state_->setHumidifierEnabled(false);
  state_->setLastControlTime(HAL_GetTick());
  state_->setTempOutput(0.0f);
  state_->setHumidityOutput(0.0f);

  // 重置PID控制器
  tempPID_->reset();
  humidityPID_->reset();
}

void TemperatureController::update(const RC::SensorData& sensorData,
                                   const RC::ControlConfig& config) noexcept {
  if (!sensorData.isValid()) return;

  const uint32_t currentTime = HAL_GetTick();
  state_->setLastControlTime(currentTime);

  switch (state_->getMode()) {
    case Mode::AUTO:
      executeAutoMode(sensorData, config);
      break;

    case Mode::MANUAL:
      executeManualMode();
      break;

    case Mode::OFF:
      // 关闭所有设备
      relayController_->turnOffAll();
      state_->setHeaterEnabled(false);
      state_->setFanEnabled(false);
      state_->setHumidifierEnabled(false);
      break;
  }
}

void TemperatureController::executeAutoMode(
    const RC::SensorData& sensorData,
    const RC::ControlConfig& config) noexcept {
  constexpr float deltaTime = 1.0f;  // 控制周期为1秒

  // 获取传感器数据
  const float currentTemp = sensorData.getTemperature();
  const float currentHumidity = sensorData.getHumidity();
  const float targetTemp = config.getTargetTemp();
  const float targetHumidity = config.getTargetHumidity();

  // PID控制计算
  const float tempOutput = tempPID_->update(targetTemp, currentTemp, deltaTime);
  const float humidityOutput =
      humidityPID_->update(targetHumidity, currentHumidity, deltaTime);

  // 更新输出值
  state_->setTempOutput(tempOutput);
  state_->setHumidityOutput(humidityOutput);

  // 执行控制逻辑
  executeTemperatureControl(currentTemp, targetTemp, config.getTempTolerance());
  executeHumidityControl(currentHumidity, targetHumidity,
                         config.getHumidityTolerance());
}

void TemperatureController::executeManualMode() noexcept {
  // 手动模式下，只更新状态记录
  using RelayType = Relay::Type;
  using RelayState = Relay::State;

  state_->setHeaterEnabled(relayController_->getState(RelayType::HEATER) ==
                           RelayState::ON);
  state_->setFanEnabled(relayController_->getState(RelayType::FAN) ==
                        RelayState::ON);
  state_->setHumidifierEnabled(
      relayController_->getState(RelayType::HUMIDIFIER) == RelayState::ON);
}

void TemperatureController::executeTemperatureControl(
    float currentTemp, float targetTemp, float tolerance) noexcept {
  using RelayType = Relay::Type;
  using RelayState = Relay::State;

  const float tempDiff = targetTemp - currentTemp;

  // 温度过低，启动加热器
  if (tempDiff > tolerance) {
    if (!state_->isHeaterEnabled()) {
      relayController_->setState(RelayType::HEATER, RelayState::ON);
      state_->setHeaterEnabled(true);
    }

    // 如果温度差异很大，也启动风扇增强对流
    if (tempDiff > 2.0f * tolerance && !state_->isFanEnabled()) {
      relayController_->setState(RelayType::FAN, RelayState::ON);
      state_->setFanEnabled(true);
    }
  }
  // 温度过高，关闭加热器，启动风扇散热
  else if (tempDiff < -tolerance) {
    if (state_->isHeaterEnabled()) {
      relayController_->setState(RelayType::HEATER, RelayState::OFF);
      state_->setHeaterEnabled(false);
    }

    if (!state_->isFanEnabled()) {
      relayController_->setState(RelayType::FAN, RelayState::ON);
      state_->setFanEnabled(true);
    }
  }
  // 温度在正常范围内
  else {
    // 温度接近目标值，关闭加热器
    if (state_->isHeaterEnabled() && tempDiff < tolerance / 2.0f) {
      relayController_->setState(RelayType::HEATER, RelayState::OFF);
      state_->setHeaterEnabled(false);
    }

    // 温度稳定，关闭风扇
    if (state_->isFanEnabled() && std::fabs(tempDiff) < tolerance / 2.0f) {
      relayController_->setState(RelayType::FAN, RelayState::OFF);
      state_->setFanEnabled(false);
    }
  }
}

void TemperatureController::executeHumidityControl(float currentHumidity,
                                                   float targetHumidity,
                                                   float tolerance) noexcept {
  using RelayType = Relay::Type;
  using RelayState = Relay::State;

  const float humidityDiff = targetHumidity - currentHumidity;

  // 湿度过低，启动雾化器
  if (humidityDiff > tolerance) {
    if (!state_->isHumidifierEnabled()) {
      relayController_->setState(RelayType::HUMIDIFIER, RelayState::ON);
      state_->setHumidifierEnabled(true);
    }
  }
  // 湿度过高，关闭雾化器，启动风扇除湿
  else if (humidityDiff < -tolerance) {
    if (state_->isHumidifierEnabled()) {
      relayController_->setState(RelayType::HUMIDIFIER, RelayState::OFF);
      state_->setHumidifierEnabled(false);
    }

    // 湿度过高时启动风扇帮助除湿
    if (humidityDiff < -2.0f * tolerance && !state_->isFanEnabled()) {
      relayController_->setState(RelayType::FAN, RelayState::ON);
      state_->setFanEnabled(true);
    }
  }
  // 湿度在正常范围内
  else {
    // 湿度接近目标值，关闭雾化器
    if (state_->isHumidifierEnabled() && humidityDiff < tolerance / 2.0f) {
      relayController_->setState(RelayType::HUMIDIFIER, RelayState::OFF);
      state_->setHumidifierEnabled(false);
    }
  }
}

void TemperatureController::setMode(Mode mode) noexcept {
  state_->setMode(mode);

  if (mode == Mode::OFF) {
    relayController_->turnOffAll();
    tempPID_->reset();
    humidityPID_->reset();
  }
}

Mode TemperatureController::getMode() const noexcept {
  return state_->getMode();
}

void TemperatureController::setTempPIDParameters(float kp, float ki,
                                                 float kd) noexcept {
  tempPID_->setParameters(kp, ki, kd);
}

void TemperatureController::setHumidityPIDParameters(float kp, float ki,
                                                     float kd) noexcept {
  humidityPID_->setParameters(kp, ki, kd);
}

void TemperatureController::setTargetTemperature(float temperature) noexcept {
  g_controlConfig.setTargetTemp(temperature);
  tempPID_->reset();  // 重置PID以避免积分饱和
}

void TemperatureController::setTargetHumidity(float humidity) noexcept {
  g_controlConfig.setTargetHumidity(humidity);
  humidityPID_->reset();  // 重置PID以避免积分饱和
}

void TemperatureController::setTolerances(float tempTolerance,
                                          float humidityTolerance) noexcept {
  g_controlConfig.setTempTolerance(tempTolerance);
  g_controlConfig.setHumidityTolerance(humidityTolerance);
}

void TemperatureController::emergencyStop() noexcept {
  // 直接关闭所有继电器
  relayController_->turnOffAll();
  state_->setHeaterEnabled(false);
  state_->setFanEnabled(false);
  state_->setHumidifierEnabled(false);
  state_->setMode(Mode::OFF);
}

}  // namespace ReptileController::Control

extern "C" {

// 兼容旧代码的类型别名
typedef RC::SensorData SensorData_t;
typedef RC::ControlConfig ControlConfig_t;

// 控制状态变量（兼容性）
static ControlState_t legacyControlState = {.mode = CONTROL_MODE_AUTO,
                                            .heaterEnabled = false,
                                            .fanEnabled = false,
                                            .humidifierEnabled = false,
                                            .lastControlTime = 0,
                                            .tempOutput = 0.0f,
                                            .humidityOutput = 0.0f};

// PID控制器（兼容性）
static PID_t legacyTempPID = {0};
static PID_t legacyHumidityPID = {0};

// 类型转换函数
static RC::Control::Mode convertToModernMode(ControlMode_t legacy) {
  switch (legacy) {
    case CONTROL_MODE_AUTO:
      return RC::Control::Mode::AUTO;
    case CONTROL_MODE_MANUAL:
      return RC::Control::Mode::MANUAL;
    case CONTROL_MODE_OFF:
      return RC::Control::Mode::OFF;
    default:
      return RC::Control::Mode::AUTO;
  }
}

static ControlMode_t convertToLegacyMode(RC::Control::Mode modern) {
  switch (modern) {
    case RC::Control::Mode::AUTO:
      return CONTROL_MODE_AUTO;
    case RC::Control::Mode::MANUAL:
      return CONTROL_MODE_MANUAL;
    case RC::Control::Mode::OFF:
      return CONTROL_MODE_OFF;
    default:
      return CONTROL_MODE_AUTO;
  }
}

static void updateLegacyState() {
  if (!RC::Control::g_controller) return;

  const auto& state = RC::Control::g_controller->getState();

  legacyControlState.mode = convertToLegacyMode(state.getMode());
  legacyControlState.heaterEnabled = state.isHeaterEnabled();
  legacyControlState.fanEnabled = state.isFanEnabled();
  legacyControlState.humidifierEnabled = state.isHumidifierEnabled();
  legacyControlState.lastControlTime = state.getLastControlTime();
  legacyControlState.tempOutput = state.getTempOutput();
  legacyControlState.humidityOutput = state.getHumidityOutput();
}

// C风格接口实现
void TempControl_Init(void) {
  // 创建全局控制器实例
  RC::Control::g_controller =
      std::make_unique<RC::Control::TemperatureController>();
  RC::Control::g_controller->initialize();

  // 初始化兼容PID结构
  PID_Init(&legacyTempPID, 2.0f, 0.1f, 0.5f);
  PID_Init(&legacyHumidityPID, 1.5f, 0.05f, 0.3f);

  updateLegacyState();
}

void TempControl_Update(SensorData_t* sensorData, ControlConfig_t* config) {
  if (!sensorData || !config || !RC::Control::g_controller) return;

  // 使用现代C++接口
  RC::Control::g_controller->update(*sensorData, *config);
  updateLegacyState();
}

void TempControl_SetMode(ControlMode_t mode) {
  if (!RC::Control::g_controller) return;

  const auto modernMode = convertToModernMode(mode);
  RC::Control::g_controller->setMode(modernMode);
  updateLegacyState();
}

ControlMode_t TempControl_GetMode(void) {
  if (!RC::Control::g_controller) return CONTROL_MODE_OFF;

  const auto modernMode = RC::Control::g_controller->getMode();
  return convertToLegacyMode(modernMode);
}

ControlState_t TempControl_GetState(void) {
  updateLegacyState();
  return legacyControlState;
}

// PID控制函数实现（兼容性）
void PID_Init(PID_t* pid, float kp, float ki, float kd) {
  if (!pid) return;

  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->integral = 0.0f;
  pid->lastError = 0.0f;
  pid->maxIntegral = 100.0f;
  pid->outputMax = 100.0f;
  pid->outputMin = -100.0f;
}

float PID_Update(PID_t* pid, float setpoint, float measurement, float dt) {
  if (!pid || dt <= 0.0f) return 0.0f;

  const float error = setpoint - measurement;

  // 比例项
  const float proportional = pid->Kp * error;

  // 积分项
  pid->integral += error * dt;
  pid->integral = clamp(pid->integral, -pid->maxIntegral, pid->maxIntegral);
  const float integral = pid->Ki * pid->integral;

  // 微分项
  const float derivative = pid->Kd * (error - pid->lastError) / dt;
  pid->lastError = error;

  // 计算输出
  const float output = proportional + integral + derivative;
  return clamp(output, pid->outputMin, pid->outputMax);
}

void PID_Reset(PID_t* pid) {
  if (!pid) return;

  pid->integral = 0.0f;
  pid->lastError = 0.0f;
}

// 控制算法函数（兼容性）
void TempControl_TemperatureControl(float currentTemp, float targetTemp,
                                    float tolerance) {
  // 这个函数的实际逻辑已经在现代C++实现中，这里只是兼容性接口
}

void TempControl_HumidityControl(float currentHumidity, float targetHumidity,
                                 float tolerance) {
  // 这个函数的实际逻辑已经在现代C++实现中，这里只是兼容性接口
}

void TempControl_AutoMode(SensorData_t* sensorData, ControlConfig_t* config) {
  if (!sensorData || !config || !RC::Control::g_controller) return;

  // 设置为自动模式并更新
  RC::Control::g_controller->setMode(RC::Control::Mode::AUTO);
  RC::Control::g_controller->update(*sensorData, *config);
}

void TempControl_ManualMode(void) {
  if (!RC::Control::g_controller) return;

  RC::Control::g_controller->setMode(RC::Control::Mode::MANUAL);
}

// 配置函数实现
void TempControl_SetTargetTemperature(float temperature) {
  if (!RC::Control::g_controller) return;

  RC::Control::g_controller->setTargetTemperature(temperature);
}

void TempControl_SetTargetHumidity(float humidity) {
  if (!RC::Control::g_controller) return;

  RC::Control::g_controller->setTargetHumidity(humidity);
}

void TempControl_SetTolerance(float tempTolerance, float humidityTolerance) {
  if (!RC::Control::g_controller) return;

  RC::Control::g_controller->setTolerances(tempTolerance, humidityTolerance);
}

}  // extern "C"