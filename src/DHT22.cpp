/*
 * DHT22 温湿度传感器驱动实现
 * DHT22.cpp
 * 现代C++风格重构版本
 */

#include "DHT22.h"
#include <array>
#include <cmath>

// 外部变量
extern "C" {
    extern TIM_HandleTypeDef htim2;
}

namespace ReptileController::DHT22 {

// 全局传感器实例
Sensor g_sensor;

// 私有方法实现
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
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // 输入模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // 上拉
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

// 公有方法实现
Status Sensor::initialize() noexcept {
    // 设置引脚为输出模式，默认高电平
    setPinOutput();
    setPin(GPIO_PIN_SET);
    HAL_Delay(1000);  // 等待DHT22稳定
    
    initialized_ = true;
    return Status::OK;
}

SensorReading Sensor::readData() noexcept {
    if (!initialized_) {
        return SensorReading(); // 返回无效数据
    }
    
    std::array<uint8_t, 5> data = {};  // 5字节数据：湿度高位、湿度低位、温度高位、温度低位、校验和
    uint32_t timeout;
    
    // 1. 发送开始信号
    setPinOutput();
    setPin(GPIO_PIN_RESET);          // 拉低1ms
    delayUs(Timing::START_SIGNAL_LOW_TIME);
    
    setPin(GPIO_PIN_SET);            // 拉高40us
    delayUs(Timing::START_SIGNAL_HIGH_TIME);
    
    // 2. 切换为输入模式，等待DHT22响应
    setPinInput();
    
    // 等待DHT22拉低（响应信号开始）
    timeout = getMicros() + Timing::TIMEOUT_US;
    while (readPin() == GPIO_PIN_SET && getMicros() < timeout) {
        // 等待拉低
    }
    if (getMicros() >= timeout) {
        return SensorReading(); // 超时，返回无效数据
    }
    
    // 等待DHT22拉高（响应信号结束）
    timeout = getMicros() + Timing::TIMEOUT_US;
    while (readPin() == GPIO_PIN_RESET && getMicros() < timeout) {
        // 等待拉高
    }
    if (getMicros() >= timeout) {
        return SensorReading(); // 超时，返回无效数据
    }
    
    // 等待DHT22拉低（数据传输开始）
    timeout = getMicros() + Timing::TIMEOUT_US;
    while (readPin() == GPIO_PIN_SET && getMicros() < timeout) {
        // 等待拉低
    }
    if (getMicros() >= timeout) {
        return SensorReading(); // 超时，返回无效数据
    }
    
    // 3. 读取40位数据
    for (int i = 0; i < 40; ++i) {
        // 等待数据位开始（低电平结束）
        timeout = getMicros() + Timing::TIMEOUT_US;
        while (readPin() == GPIO_PIN_RESET && getMicros() < timeout) {
            // 等待拉高
        }
        if (getMicros() >= timeout) {
            return SensorReading(); // 超时，返回无效数据
        }
        
        // 测量高电平时间
        const uint32_t startTime = getMicros();
        timeout = startTime + Timing::TIMEOUT_US;
        while (readPin() == GPIO_PIN_SET && getMicros() < timeout) {
            // 等待拉低
        }
        if (getMicros() >= timeout) {
            return SensorReading(); // 超时，返回无效数据
        }
        
        const uint32_t highTime = getMicros() - startTime;
        
        // 根据高电平时间判断数据位
        data[i / 8] <<= 1;
        if (highTime > Timing::BIT_HIGH_TIME_THRESHOLD) {
            data[i / 8] |= 1;  // 数据位为1
        }
        // 数据位为0时不需要额外操作
    }
    
    // 4. 校验数据
    const uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        return SensorReading(); // 校验和错误，返回无效数据
    }
    
    // 5. 转换数据
    // 湿度数据 (0.1%精度)
    const uint16_t humidityRaw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    float humidity = static_cast<float>(humidityRaw) / 10.0f;
    
    // 温度数据 (0.1°C精度)
    uint16_t temperatureRaw = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    float temperature;
    
    if (temperatureRaw & 0x8000) {  // 负温度
        temperatureRaw &= 0x7FFF;
        temperature = -static_cast<float>(temperatureRaw) / 10.0f;
    } else {
        temperature = static_cast<float>(temperatureRaw) / 10.0f;
    }
    
    // 数据范围检查
    if (humidity < Range::MIN_HUMIDITY || humidity > Range::MAX_HUMIDITY ||
        temperature < Range::MIN_TEMPERATURE || temperature > Range::MAX_TEMPERATURE) {
        return SensorReading(); // 数据超出范围，返回无效数据
    }
    
    return SensorReading(temperature, humidity); // 返回有效数据
}

Status Sensor::readTemperatureHumidity(float* temperature, float* humidity) noexcept {
    if (!temperature || !humidity) {
        return Status::NO_RESPONSE;
    }
    
    SensorReading reading = readData();
    if (reading.valid) {
        *temperature = reading.temperature;
        *humidity = reading.humidity;
        return Status::OK;
    }
    
    return Status::NO_RESPONSE;
}

} // namespace ReptileController::DHT22

extern "C" {

// C风格接口实现（为了兼容性）
DHT22_Status_t DHT22_Init(void) {
    const auto status = ReptileController::DHT22::g_sensor.initialize();
    switch (status) {
        case ReptileController::DHT22::Status::OK:
            return DHT22_OK;
        case ReptileController::DHT22::Status::TIMEOUT:
            return DHT22_ERROR_TIMEOUT;
        case ReptileController::DHT22::Status::CHECKSUM_ERROR:
            return DHT22_ERROR_CHECKSUM;
        case ReptileController::DHT22::Status::NO_RESPONSE:
            return DHT22_ERROR_NO_RESPONSE;
        default:
            return DHT22_ERROR_NO_RESPONSE;
    }
}

DHT22_Status_t DHT22_ReadData(float *temperature, float *humidity) {
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

// 兼容旧接口的函数
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

} // extern "C" 