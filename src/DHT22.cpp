/*
 * DHT22 温湿度传感器驱动实现
 * DHT22.cpp
 * 现代C++风格重构版本
 */

#include "DHT22.h"

#include <array>
#include <cmath>

extern "C" {
extern TIM_HandleTypeDef htim2;
}

namespace ReptileController::DHT22 {

Sensor g_sensor;

void Sensor::setPinOutput() const noexcept {
  GPIO_InitTypeDef GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = Pins::DHT22::PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // 上拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Pins::DHT22::PORT, &GPIO_InitStruct);
}

void Sensor::setPinInput() const noexcept {
  GPIO_InitTypeDef GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = Pins::DHT22::PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // 输入模式
  GPIO_InitStruct.Pull = GPIO_PULLUP;      // 上拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Pins::DHT22::PORT, &GPIO_InitStruct);
}

void Sensor::setPin(GPIO_PinState state) const noexcept {
  HAL_GPIO_WritePin(Pins::DHT22::PORT, Pins::DHT22::PIN, state);
}

GPIO_PinState Sensor::readPin() const noexcept {
  return HAL_GPIO_ReadPin(Pins::DHT22::PORT, Pins::DHT22::PIN);
}

void Sensor::delayUs(uint32_t us) const noexcept {
  const uint32_t startTime = getMicros();
  while ((getMicros() - startTime) < us) {
    // 空循环等待
  }
}

uint32_t Sensor::getMicros() const noexcept {
  return __HAL_TIM_GET_COUNTER(&htim2);
}

auto Sensor::initialize() noexcept -> DHT22Status {
  setPinOutput();
  setPin(GPIO_PIN_SET);
  HAL_Delay(1000);
  initialized_ = true;
  return DHT22Status::OK;
}

auto Sensor::readData() noexcept -> SensorReading {
  if (!initialized_) {
    return SensorReading();
  }
  std::array<uint8_t, 5> data = {};
  uint32_t timeout;

  setPinOutput();
  setPin(GPIO_PIN_RESET);
  delayUs(Timing::START_SIGNAL_LOW_TIME);

  setPin(GPIO_PIN_SET);
  delayUs(Timing::START_SIGNAL_HIGH_TIME);

  setPinInput();
  timeout = getMicros() + Timing::TIMEOUT_US;
  while (readPin() == GPIO_PIN_SET && getMicros() < timeout) {
  }
  if (getMicros() >= timeout) {
    return SensorReading();
  }

  timeout = getMicros() + Timing::TIMEOUT_US;
  while (readPin() == GPIO_PIN_RESET && getMicros() < timeout) {
  }
  if (getMicros() >= timeout) {
    return SensorReading();
  }

  timeout = getMicros() + Timing::TIMEOUT_US;
  while (readPin() == GPIO_PIN_SET && getMicros() < timeout) {
  }
  if (getMicros() >= timeout) {
    return SensorReading();
  }

  for (int i = 0; i < 40; ++i) {
    timeout = getMicros() + Timing::TIMEOUT_US;
    while (readPin() == GPIO_PIN_RESET && getMicros() < timeout) {
    }
    if (getMicros() >= timeout) {
      return SensorReading();
    }

    const uint32_t startTime = getMicros();
    timeout = startTime + Timing::TIMEOUT_US;
    while (readPin() == GPIO_PIN_SET && getMicros() < timeout) {
    }
    if (getMicros() >= timeout) {
      return SensorReading();
    }

    const uint32_t highTime = getMicros() - startTime;

    data[i / 8] <<= 1;
    if (highTime > Timing::BIT_HIGH_TIME_THRESHOLD) {
      data[i / 8] |= 1;
    }
  }

  const uint8_t checksum = data[0] + data[1] + data[2] + data[3];
  if (checksum != data[4]) {
    return SensorReading();
  }

  const uint16_t humidityRaw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  float humidity = static_cast<float>(humidityRaw) / 10.0f;

  uint16_t temperatureRaw = (static_cast<uint16_t>(data[2]) << 8) | data[3];
  float temperature;

  if (temperatureRaw & 0x8000) {
    temperatureRaw &= 0x7FFF;
    temperature = -static_cast<float>(temperatureRaw) / 10.0f;
  } else {
    temperature = static_cast<float>(temperatureRaw) / 10.0f;
  }

  if (humidity < Range::MIN_HUMIDITY || humidity > Range::MAX_HUMIDITY ||
      temperature < Range::MIN_TEMPERATURE ||
      temperature > Range::MAX_TEMPERATURE) {
    return SensorReading();
  }

  return SensorReading(temperature, humidity);
}

auto Sensor::readTemperatureHumidity(float* temperature,
                                     float* humidity) noexcept -> DHT22Status {
  if (!temperature || !humidity) {
    return DHT22Status::NO_RESPONSE;
  }

  SensorReading reading = readData();
  if (reading.valid) {
    *temperature = reading.temperature;
    *humidity = reading.humidity;
    return DHT22Status::OK;
  }

  return DHT22Status::NO_RESPONSE;
}

}  // namespace ReptileController::DHT22

extern "C" {

DHT22_Status_t DHT22_Init(void) {
  const auto status = ReptileController::DHT22::g_sensor.initialize();
  switch (status) {
    case ReptileController::DHT22::DHT22Status::OK:
      return DHT22_OK;
    case ReptileController::DHT22::DHT22Status::TIMEOUT:
      return DHT22_ERROR_TIMEOUT;
    case ReptileController::DHT22::DHT22Status::CHECKSUM_ERROR:
      return DHT22_ERROR_CHECKSUM;
    case ReptileController::DHT22::DHT22Status::NO_RESPONSE:
      return DHT22_ERROR_NO_RESPONSE;
    default:
      return DHT22_ERROR_NO_RESPONSE;
  }
}

DHT22_Status_t DHT22_ReadData(float* temperature, float* humidity) {
  if (!temperature || !humidity) {
    return DHT22_ERROR_NO_RESPONSE;
  }

  const auto result = ReptileController::DHT22::g_sensor.readData();
  if (result.valid) {
    *temperature = result.temperature;
    *humidity = result.humidity;
    return DHT22_OK;
  }

  return DHT22_ERROR_NO_RESPONSE;
}

void DHT22_SetPinOutput(void) {
  ReptileController::DHT22::g_sensor.setPinOutput();
}
void DHT22_SetPinInput(void) {
  ReptileController::DHT22::g_sensor.setPinInput();
}
void DHT22_SetPin(GPIO_PinState state) {
  ReptileController::DHT22::g_sensor.setPin(state);
}
GPIO_PinState DHT22_ReadPin(void) {
  return ReptileController::DHT22::g_sensor.readPin();
}
void DHT22_DelayUs(uint32_t us) {
  ReptileController::DHT22::g_sensor.delayUs(us);
}
uint32_t DHT22_GetMicros(void) {
  return ReptileController::DHT22::g_sensor.getMicros();
}
}  // extern "C"