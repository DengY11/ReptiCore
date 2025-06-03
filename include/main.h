/*
 * STM32F407 温湿度智能控制系统
 * 主头文件 - main.h
 * 现代C++风格重构版本
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// STM32 HAL库
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"

#ifdef __cplusplus
}

// C++标准库
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>

namespace ReptileController {

// 系统配置常量
namespace Config {
constexpr uint32_t HSE_FREQUENCY = 25'000'000U;  // 外部晶振频率 25MHz
constexpr uint32_t HSI_FREQUENCY = 16'000'000U;  // 内部RC频率 16MHz
constexpr uint8_t OLED_I2C_ADDRESS = 0x78;       // SSD1306 OLED I2C地址
}  // namespace Config

// GPIO引脚配置
namespace Pins {
namespace DHT22 {
constexpr auto PORT = GPIOA;
constexpr auto PIN = GPIO_PIN_1;
}  // namespace DHT22

namespace Relay {
constexpr auto HEATER_PORT = GPIOC;
constexpr auto HEATER_PIN = GPIO_PIN_13;

constexpr auto FAN_PORT = GPIOC;
constexpr auto FAN_PIN = GPIO_PIN_14;

constexpr auto HUMIDIFIER_PORT = GPIOC;
constexpr auto HUMIDIFIER_PIN = GPIO_PIN_15;
}  // namespace Relay
}  // namespace Pins

// 传感器数据类
class SensorData {
 private:
  float temperature_{0.0f};     // 温度 (°C)
  float humidity_{0.0f};        // 湿度 (%)
  uint32_t lastUpdateTime_{0};  // 最后更新时间
  bool isValid_{false};         // 数据有效性

 public:
  SensorData() = default;

  // 使用统一初始化
  SensorData(float temp, float hum, uint32_t updateTime, bool valid = true)
      : temperature_{temp},
        humidity_{hum},
        lastUpdateTime_{updateTime},
        isValid_{valid} {}

  // Getter方法（const正确性）
  [[nodiscard]] float getTemperature() const noexcept { return temperature_; }
  [[nodiscard]] float getHumidity() const noexcept { return humidity_; }
  [[nodiscard]] uint32_t getLastUpdateTime() const noexcept {
    return lastUpdateTime_;
  }
  [[nodiscard]] bool isValid() const noexcept { return isValid_; }

  // Setter方法
  void setTemperature(float temp) noexcept { temperature_ = temp; }
  void setHumidity(float hum) noexcept { humidity_ = hum; }
  void setLastUpdateTime(uint32_t time) noexcept { lastUpdateTime_ = time; }
  void setValid(bool valid) noexcept { isValid_ = valid; }

  // 更新所有数据
  void updateData(float temp, float hum, uint32_t time) noexcept {
    temperature_ = temp;
    humidity_ = hum;
    lastUpdateTime_ = time;
    isValid_ = true;
  }

  // 标记数据无效
  void invalidate() noexcept { isValid_ = false; }
};

// 控制配置类
class ControlConfig {
 private:
  float targetTemp_{25.0f};        // 目标温度
  float targetHumidity_{60.0f};    // 目标湿度
  float tempTolerance_{1.0f};      // 温度容差
  float humidityTolerance_{5.0f};  // 湿度容差

 public:
  ControlConfig() = default;

  // 使用统一初始化
  ControlConfig(float temp, float humidity, float tempTol, float humTol)
      : targetTemp_{temp},
        targetHumidity_{humidity},
        tempTolerance_{tempTol},
        humidityTolerance_{humTol} {}

  // Getter方法（const正确性）
  [[nodiscard]] float getTargetTemp() const noexcept { return targetTemp_; }
  [[nodiscard]] float getTargetHumidity() const noexcept {
    return targetHumidity_;
  }
  [[nodiscard]] float getTempTolerance() const noexcept {
    return tempTolerance_;
  }
  [[nodiscard]] float getHumidityTolerance() const noexcept {
    return humidityTolerance_;
  }

  // Setter方法（范围检查）
  void setTargetTemp(float temp) noexcept {
    if (temp >= -40.0f && temp <= 80.0f) {
      targetTemp_ = temp;
    }
  }

  void setTargetHumidity(float humidity) noexcept {
    if (humidity >= 0.0f && humidity <= 100.0f) {
      targetHumidity_ = humidity;
    }
  }

  void setTempTolerance(float tolerance) noexcept {
    if (tolerance > 0.0f && tolerance <= 10.0f) {
      tempTolerance_ = tolerance;
    }
  }

  void setHumidityTolerance(float tolerance) noexcept {
    if (tolerance > 0.0f && tolerance <= 20.0f) {
      humidityTolerance_ = tolerance;
    }
  }
};

// 全局变量声明（使用extern）
extern SensorData g_sensorData;
extern ControlConfig g_controlConfig;

}  // namespace ReptileController

extern "C" {
#endif

// C函数声明
void Error_Handler(void);
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif