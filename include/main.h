/*
 * STM32F407 温湿度智能控制系统
 * 主头文件 - main.h
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// STM32 HAL库
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_tim.h"

// C标准库
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// 系统配置
#define HSE_VALUE    25000000U  // 外部晶振频率 25MHz
#define HSI_VALUE    16000000U  // 内部RC频率 16MHz

// GPIO引脚定义
#define DHT22_GPIO_Port       GPIOA
#define DHT22_Pin             GPIO_PIN_1

#define RELAY_HEATER_Port     GPIOC
#define RELAY_HEATER_Pin      GPIO_PIN_13

#define RELAY_FAN_Port        GPIOC  
#define RELAY_FAN_Pin         GPIO_PIN_14

#define RELAY_HUMIDIFIER_Port GPIOC
#define RELAY_HUMIDIFIER_Pin  GPIO_PIN_15

// I2C OLED地址
#define OLED_I2C_ADDRESS      0x78  // SSD1306 OLED I2C地址

// 传感器数据结构
typedef struct {
    float temperature;      // 温度 (°C)
    float humidity;         // 湿度 (%)
    uint32_t lastUpdateTime; // 最后更新时间
    bool isValid;           // 数据有效性
} SensorData_t;

// 控制配置结构
typedef struct {
    float targetTemp;       // 目标温度
    float targetHumidity;   // 目标湿度
    float tempTolerance;    // 温度容差
    float humidityTolerance; // 湿度容差
} ControlConfig_t;

// 全局变量声明
extern SensorData_t sensorData;
extern ControlConfig_t controlConfig;

// 函数声明
void Error_Handler(void);
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */ 