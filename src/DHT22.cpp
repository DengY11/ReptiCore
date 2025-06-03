/*
 * DHT22 温湿度传感器驱动实现
 * DHT22.cpp
 */

#include "DHT22.h"

// 外部变量
extern TIM_HandleTypeDef htim2;

// DHT22初始化
DHT22_Status_t DHT22_Init(void)
{
    // 设置引脚为输出模式，默认高电平
    DHT22_SetPinOutput();
    DHT22_SetPin(GPIO_PIN_SET);
    HAL_Delay(1000);  // 等待DHT22稳定
    
    return DHT22_OK;
}

// 读取DHT22数据
DHT22_Status_t DHT22_ReadData(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};  // 5字节数据：湿度高位、湿度低位、温度高位、温度低位、校验和
    uint8_t checksum = 0;
    uint32_t timeout;
    
    // 1. 发送开始信号
    DHT22_SetPinOutput();
    DHT22_SetPin(GPIO_PIN_RESET);          // 拉低1ms
    DHT22_DelayUs(DHT22_START_SIGNAL_LOW_TIME);
    
    DHT22_SetPin(GPIO_PIN_SET);            // 拉高40us
    DHT22_DelayUs(DHT22_START_SIGNAL_HIGH_TIME);
    
    // 2. 切换为输入模式，等待DHT22响应
    DHT22_SetPinInput();
    
    // 等待DHT22拉低（响应信号开始）
    timeout = DHT22_GetMicros() + DHT22_TIMEOUT_US;
    while (DHT22_ReadPin() == GPIO_PIN_SET && DHT22_GetMicros() < timeout);
    if (DHT22_GetMicros() >= timeout) {
        return DHT22_ERROR_NO_RESPONSE;
    }
    
    // 等待DHT22拉高（响应信号结束）
    timeout = DHT22_GetMicros() + DHT22_TIMEOUT_US;
    while (DHT22_ReadPin() == GPIO_PIN_RESET && DHT22_GetMicros() < timeout);
    if (DHT22_GetMicros() >= timeout) {
        return DHT22_ERROR_NO_RESPONSE;
    }
    
    // 等待DHT22拉低（数据传输开始）
    timeout = DHT22_GetMicros() + DHT22_TIMEOUT_US;
    while (DHT22_ReadPin() == GPIO_PIN_SET && DHT22_GetMicros() < timeout);
    if (DHT22_GetMicros() >= timeout) {
        return DHT22_ERROR_NO_RESPONSE;
    }
    
    // 3. 读取40位数据
    for (int i = 0; i < 40; i++) {
        uint32_t highTime;
        
        // 等待数据位开始（低电平结束）
        timeout = DHT22_GetMicros() + DHT22_TIMEOUT_US;
        while (DHT22_ReadPin() == GPIO_PIN_RESET && DHT22_GetMicros() < timeout);
        if (DHT22_GetMicros() >= timeout) {
            return DHT22_ERROR_TIMEOUT;
        }
        
        // 测量高电平时间
        uint32_t startTime = DHT22_GetMicros();
        timeout = startTime + DHT22_TIMEOUT_US;
        while (DHT22_ReadPin() == GPIO_PIN_SET && DHT22_GetMicros() < timeout);
        if (DHT22_GetMicros() >= timeout) {
            return DHT22_ERROR_TIMEOUT;
        }
        
        highTime = DHT22_GetMicros() - startTime;
        
        // 根据高电平时间判断数据位
        data[i / 8] <<= 1;
        if (highTime > DHT22_BIT_HIGH_TIME_THRESHOLD) {
            data[i / 8] |= 1;  // 数据位为1
        }
        // 数据位为0时不需要额外操作
    }
    
    // 4. 校验数据
    checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        return DHT22_ERROR_CHECKSUM;
    }
    
    // 5. 转换数据
    // 湿度数据 (0.1%精度)
    uint16_t humidityRaw = (data[0] << 8) | data[1];
    *humidity = (float)humidityRaw / 10.0f;
    
    // 温度数据 (0.1°C精度)
    uint16_t temperatureRaw = (data[2] << 8) | data[3];
    if (temperatureRaw & 0x8000) {  // 负温度
        temperatureRaw &= 0x7FFF;
        *temperature = -(float)temperatureRaw / 10.0f;
    } else {
        *temperature = (float)temperatureRaw / 10.0f;
    }
    
    // 数据范围检查
    if (*humidity < 0 || *humidity > 100 || *temperature < -40 || *temperature > 80) {
        return DHT22_ERROR_CHECKSUM;
    }
    
    return DHT22_OK;
}

// 设置引脚为输出模式
void DHT22_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}

// 设置引脚为输入模式
void DHT22_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // 输入模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}

// 设置引脚电平
void DHT22_SetPin(GPIO_PinState state)
{
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, state);
}

// 读取引脚电平
GPIO_PinState DHT22_ReadPin(void)
{
    return HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin);
}

// 微秒延时
void DHT22_DelayUs(uint32_t us)
{
    uint32_t startTime = DHT22_GetMicros();
    while ((DHT22_GetMicros() - startTime) < us);
}

// 获取微秒计数
uint32_t DHT22_GetMicros(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
} 