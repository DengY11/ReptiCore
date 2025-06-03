/*
 * 温湿度智能控制算法实现
 * TempControl.cpp
 * 现代C++17风格重构版本
 */

#include "TempControl.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "RelayControl.h"

// 命名空间别名
namespace RC = ReptileController;

// 全局实例声明
extern RC::SensorData g_sensorData;
extern RC::ControlConfig g_controlConfig;

namespace ReptileController::Control {

// 温控器全局实例
std::unique_ptr<TemperatureController> g_controller = nullptr;

// 温控器构造函数
TemperatureController::TemperatureController()
    : tempPID_(std::make_unique<PIDController<float>>(2.0f, 0.5f, 0.1f)),
      humidityPID_(std::make_unique<PIDController<float>>(1.5f, 0.3f, 0.05f)),
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
  if (!relayController_) return;
  
  relayController_->initialize();
  state_->setMode(Mode::AUTO);
  state_->setLastControlTime(HAL_GetTick());
}

void TemperatureController::update(const RC::SensorData& sensorData,
                                   const RC::ControlConfig& config) noexcept {
  if (!sensorData.isValid() || !state_) return;
  
  const uint32_t currentTime = HAL_GetTick();
  const uint32_t deltaTime = currentTime - state_->getLastControlTime();
  
  // 至少间隔1秒才执行控制
  if (deltaTime < 1000) return;
  
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
      if (relayController_) {
        relayController_->turnOffAll();
      }
      break;
  }
  
  // 执行安全检查
  if (relayController_) {
    relayController_->safetyCheck();
  }
}

void TemperatureController::executeAutoMode(
    const RC::SensorData& sensorData,
    const RC::ControlConfig& config) noexcept {
  const float currentTemp = sensorData.getTemperature();
  const float currentHumidity = sensorData.getHumidity();
  const float targetTemp = config.getTargetTemp();
  const float targetHumidity = config.getTargetHumidity();
  const float tempTolerance = config.getTempTolerance();
  const float humidityTolerance = config.getHumidityTolerance();
  
  executeTemperatureControl(currentTemp, targetTemp, tempTolerance);
  executeHumidityControl(currentHumidity, targetHumidity, humidityTolerance);
}

void TemperatureController::executeManualMode() noexcept {
  // 手动模式下不自动控制，保持当前状态
  // 可以通过其他接口手动控制继电器
}

void TemperatureController::executeTemperatureControl(
    float currentTemp, float targetTemp, float tolerance) noexcept {
  if (!tempPID_ || !relayController_ || !state_) return;
  
  const float tempError = targetTemp - currentTemp;
  const float deltaTime = 1.0f;  // 1秒间隔
  
  // 使用PID控制器计算输出
  const float pidOutput = tempPID_->update(targetTemp, currentTemp, deltaTime);
  state_->setTempOutput(pidOutput);
  
  // 简单的温度控制逻辑
  if (tempError > tolerance) {
    // 温度太低，需要加热
    relayController_->setState(Relay::Type::HEATER, Relay::State::ON);
    relayController_->setState(Relay::Type::FAN, Relay::State::OFF);
    state_->setHeaterEnabled(true);
    state_->setFanEnabled(false);
  } else if (tempError < -tolerance) {
    // 温度太高，需要降温
    relayController_->setState(Relay::Type::HEATER, Relay::State::OFF);
    relayController_->setState(Relay::Type::FAN, Relay::State::ON);
    state_->setHeaterEnabled(false);
    state_->setFanEnabled(true);
  } else {
    // 温度在目标范围内
    relayController_->setState(Relay::Type::HEATER, Relay::State::OFF);
    relayController_->setState(Relay::Type::FAN, Relay::State::OFF);
    state_->setHeaterEnabled(false);
    state_->setFanEnabled(false);
  }
}

void TemperatureController::executeHumidityControl(float currentHumidity,
                                                   float targetHumidity,
                                                   float tolerance) noexcept {
  if (!humidityPID_ || !relayController_ || !state_) return;
  
  const float humidityError = targetHumidity - currentHumidity;
  const float deltaTime = 1.0f;  // 1秒间隔
  
  // 使用PID控制器计算输出
  const float pidOutput = humidityPID_->update(targetHumidity, currentHumidity, deltaTime);
  state_->setHumidityOutput(pidOutput);
  
  // 简单的湿度控制逻辑
  if (humidityError > tolerance) {
    // 湿度太低，需要加湿
    relayController_->setState(Relay::Type::HUMIDIFIER, Relay::State::ON);
    state_->setHumidifierEnabled(true);
  } else {
    // 湿度合适或太高，关闭加湿器
    relayController_->setState(Relay::Type::HUMIDIFIER, Relay::State::OFF);
    state_->setHumidifierEnabled(false);
  }
}

void TemperatureController::setMode(Mode mode) noexcept {
  if (state_) {
    state_->setMode(mode);
  }
}

Mode TemperatureController::getMode() const noexcept {
  return state_ ? state_->getMode() : Mode::OFF;
}

void TemperatureController::setTemperaturePIDParameters(float kp, float ki,
                                                 float kd) noexcept {
  if (tempPID_) {
    tempPID_->setParameters(kp, ki, kd);
  }
}

void TemperatureController::setHumidityPIDParameters(float kp, float ki,
                                                     float kd) noexcept {
  if (humidityPID_) {
    humidityPID_->setParameters(kp, ki, kd);
  }
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
  if (relayController_) {
    relayController_->emergencyStop();
  }
  if (state_) {
    state_->setMode(Mode::OFF);
  }
}

bool TemperatureController::safetyCheck() const noexcept {
  if (!relayController_) return false;
  
  // 检查继电器控制器是否正常
  return relayController_->isInitialized();
}

}  // namespace ReptileController::Control

// 简单的clamp函数实现（在extern C块外面）
template<typename T>
static T clamp(const T& value, const T& min, const T& max) {
  return (value < min) ? min : (value > max) ? max : value;
}

extern "C" {

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

// 转换C结构体到C++类
static RC::SensorData convertToModernSensorData(const SensorData_t* legacy) {
  RC::SensorData modern;
  if (legacy) {
    modern.updateData(legacy->temperature, legacy->humidity, legacy->timestamp);
    if (!legacy->valid) {
      modern.invalidate();
    }
  }
  return modern;
}

static RC::ControlConfig convertToModernControlConfig(const ControlConfig_t* legacy) {
  RC::ControlConfig modern;
  if (legacy) {
    modern.setTargetTemp(legacy->targetTemperature);
    modern.setTargetHumidity(legacy->targetHumidity);
    modern.setTempTolerance(legacy->tempTolerance);
    modern.setHumidityTolerance(legacy->humidityTolerance);
  }
  return modern;
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
  PID_Init(&legacyTempPID, 2.0f, 0.5f, 0.1f);
  PID_Init(&legacyHumidityPID, 1.5f, 0.3f, 0.05f);

  updateLegacyState();
}

void TempControl_Update(SensorData_t* sensorData, ControlConfig_t* config) {
  if (!sensorData || !config || !RC::Control::g_controller) return;

  // 转换C结构体到C++类
  auto modernSensorData = convertToModernSensorData(sensorData);
  auto modernConfig = convertToModernControlConfig(config);

  // 使用现代C++接口
  RC::Control::g_controller->update(modernSensorData, modernConfig);
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

  // 转换C结构体到C++类
  auto modernSensorData = convertToModernSensorData(sensorData);
  auto modernConfig = convertToModernControlConfig(config);

  // 设置为自动模式并更新
  RC::Control::g_controller->setMode(RC::Control::Mode::AUTO);
  RC::Control::g_controller->update(modernSensorData, modernConfig);
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