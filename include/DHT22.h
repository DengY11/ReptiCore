/*
 * DHT22 温湿度传感器驱动头文件
 * DHT22.h
 * 现代C++风格重构版本
 */

#pragma once

#include <cstdint>
#include <utility>

#include "main.h"

// 为了兼容性，在某些情况下可能没有std::optional
#if __cplusplus >= 201703L
#include <optional>
#define HAS_OPTIONAL 1
#else
#define HAS_OPTIONAL 0
#endif

namespace ReptileController::DHT22 {

// DHT22返回状态
enum class DHT22Status : uint8_t {
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

namespace Range {
constexpr float MIN_TEMPERATURE = -40.0f;
constexpr float MAX_TEMPERATURE = 80.0f;
constexpr float MIN_HUMIDITY = 0.0f;
constexpr float MAX_HUMIDITY = 100.0f;
}  // namespace Range

struct SensorReading {
  float temperature;
  float humidity;
  bool valid;

  SensorReading() : temperature(0.0f), humidity(0.0f), valid(false) {}
  SensorReading(float temp, float hum)
      : temperature(temp), humidity(hum), valid(true) {}
};

class Sensor {
 private:
  bool initialized_{false};

 public:
  Sensor() = default;
  ~Sensor() = default;
  Sensor(const Sensor&) = delete;
  Sensor& operator=(const Sensor&) = delete;

  Sensor(Sensor&&) = default;
  Sensor& operator=(Sensor&&) = default;
  DHT22Status initialize() noexcept;
  auto readData() noexcept -> SensorReading;
  bool isInitialized() const noexcept { return initialized_; }

  auto readTemperatureHumidity(float* temperature, float* humidity) noexcept
      -> DHT22Status;

  /*
    为了c的兼容性
    提供公有方法
  */
  void setPinOutput() const noexcept;
  void setPinInput() const noexcept;
  void setPin(GPIO_PinState state) const noexcept;
  GPIO_PinState readPin() const noexcept;
  void delayUs(uint32_t us) const noexcept;
  uint32_t getMicros() const noexcept;
};

extern Sensor g_sensor;

}  // namespace ReptileController::DHT22

// C接口部分
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DHT22_OK = 0,
  DHT22_ERROR_TIMEOUT,
  DHT22_ERROR_CHECKSUM,
  DHT22_ERROR_NO_RESPONSE
} DHT22_Status_t;

DHT22_Status_t DHT22_Init(void);
DHT22_Status_t DHT22_ReadData(float* temperature, float* humidity);

void DHT22_SetPinOutput(void);
void DHT22_SetPinInput(void);
void DHT22_SetPin(GPIO_PinState state);
GPIO_PinState DHT22_ReadPin(void);
void DHT22_DelayUs(uint32_t us);
uint32_t DHT22_GetMicros(void);

#ifdef __cplusplus
}
#endif