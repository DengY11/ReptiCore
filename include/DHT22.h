/*
 * DHT22 温湿度传感器驱动头文件
 * DHT22.h
 */

#ifndef __DHT22_H
#define __DHT22_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// DHT22返回状态枚举
typedef enum {
    DHT22_OK = 0,           // 读取成功
    DHT22_ERROR_TIMEOUT,    // 超时错误
    DHT22_ERROR_CHECKSUM,   // 校验和错误
    DHT22_ERROR_NO_RESPONSE // 无响应错误
} DHT22_Status_t;

// DHT22时序参数 (微秒)
#define DHT22_START_SIGNAL_LOW_TIME     1000    // 开始信号低电平时间
#define DHT22_START_SIGNAL_HIGH_TIME    40      // 开始信号高电平时间
#define DHT22_RESPONSE_WAIT_TIME        80      // 等待响应时间
#define DHT22_RESPONSE_LOW_TIME         80      // 响应低电平时间
#define DHT22_RESPONSE_HIGH_TIME        80      // 响应高电平时间
#define DHT22_BIT_HIGH_TIME_THRESHOLD   40      // 数据位高电平时间阈值
#define DHT22_TIMEOUT_US                1000    // 超时时间

// 函数声明
DHT22_Status_t DHT22_Init(void);
DHT22_Status_t DHT22_ReadData(float *temperature, float *humidity);

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

#endif /* __DHT22_H */ 