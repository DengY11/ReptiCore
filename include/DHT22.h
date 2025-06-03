/*
 * DHT22 温湿度传感器驱动头文件
 * DHT22.h
 * 现代C++风格重构版本
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}

#include <cstdint>
#include <utility>

// 为了兼容性，在某些情况下可能没有std::optional
#if __cplusplus >= 201703L
#include <optional>
#define HAS_OPTIONAL 1
#else
#define HAS_OPTIONAL 0
#endif

namespace ReptileController::DHT22 {

// DHT22返回状态枚举（使用enum class）
enum class Status : uint8_t {
  OK = 0,          // 读取成功
  TIMEOUT,         // 超时错误
  CHECKSUM_ERROR,  // 校验和错误
  NO_RESPONSE      // 无响应错误
};

// DHT22时序参数 (微秒) - 使用constexpr
namespace Timing {
constexpr uint32_t START_SIGNAL_LOW_TIME = 1000;  // 开始信号低电平时间
constexpr uint32_t START_SIGNAL_HIGH_TIME = 40;   // 开始信号高电平时间
constexpr uint32_t RESPONSE_WAIT_TIME = 80;       // 等待响应时间
constexpr uint32_t RESPONSE_LOW_TIME = 80;        // 响应低电平时间
constexpr uint32_t RESPONSE_HIGH_TIME = 80;       // 响应高电平时间
constexpr uint32_t BIT_HIGH_TIME_THRESHOLD = 40;  // 数据位高电平时间阈值
constexpr uint32_t TIMEOUT_US = 1000;             // 超时时间
}  // namespace Timing

// 传感器数据范围
namespace Range {
constexpr float MIN_TEMPERATURE = -40.0f;
constexpr float MAX_TEMPERATURE = 80.0f;
constexpr float MIN_HUMIDITY = 0.0f;
constexpr float MAX_HUMIDITY = 100.0f;
}  // namespace Range

// 传感器数据结构
struct SensorReading {
  float temperature;
  float humidity;
  bool valid;

  SensorReading() : temperature(0.0f), humidity(0.0f), valid(false) {}
  SensorReading(float temp, float hum)
      : temperature(temp), humidity(hum), valid(true) {}
};

// DHT22传感器类
class Sensor {
 private:
  bool initialized_{false};

 public:
  Sensor() = default;
  ~Sensor() = default;

  // 禁用拷贝构造和赋值（传感器是唯一的）
  Sensor(const Sensor&) = delete;
  Sensor& operator=(const Sensor&) = delete;

  // 允许移动构造和赋值
  Sensor(Sensor&&) = default;
  Sensor& operator=(Sensor&&) = default;

  // 初始化传感器
  Status initialize() noexcept;

  // 读取传感器数据（返回结构体）
  SensorReading readData() noexcept;

  // 检查是否已初始化
  bool isInitialized() const noexcept { return initialized_; }

  // 为了兼容性，提供分离的读取函数
  Status readTemperatureHumidity(float* temperature, float* humidity) noexcept;

  // 公有方法（为了C接口兼容性）
  void setPinOutput() const noexcept;
  void setPinInput() const noexcept;
  void setPin(GPIO_PinState state) const noexcept;
  GPIO_PinState readPin() const noexcept;
  void delayUs(uint32_t us) const noexcept;
  uint32_t getMicros() const noexcept;
};

// 全局传感器实例
extern Sensor g_sensor;

}  // namespace ReptileController::DHT22

extern "C" {
#endif

// C风格接口（为了兼容性）
typedef enum {
  DHT22_OK = 0,
  DHT22_ERROR_TIMEOUT,
  DHT22_ERROR_CHECKSUM,
  DHT22_ERROR_NO_RESPONSE
} DHT22_Status_t;

// C函数声明
DHT22_Status_t DHT22_Init(void);
DHT22_Status_t DHT22_ReadData(float* temperature, float* humidity);

// 内部函数
void DHT22_SetPinOutput(void);
void DHT22_SetPinInput(void);
void DHT22_SetPin(GPIO_PinState state);
GPIO_PinState DHT22_ReadPin(void);
void DHT22_DelayUs(uint32_t us);
uint32_t DHT22_GetMicros(void);

#ifdef __cplusplus
}
#endif