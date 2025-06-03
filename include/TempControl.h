/*
 * 温湿度智能控制算法头文件
 * TempControl.h
 */

#ifndef __TEMP_CONTROL_H
#define __TEMP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "RelayControl.h"

// 控制模式枚举
typedef enum {
    CONTROL_MODE_AUTO = 0,      // 自动模式
    CONTROL_MODE_MANUAL,        // 手动模式
    CONTROL_MODE_OFF            // 关闭模式
} ControlMode_t;

// PID控制参数结构
typedef struct {
    float Kp;                   // 比例系数
    float Ki;                   // 积分系数
    float Kd;                   // 微分系数
    float integral;             // 积分累积值
    float lastError;            // 上次误差值
    float maxIntegral;          // 积分限幅
    float outputMax;            // 输出最大值
    float outputMin;            // 输出最小值
} PID_t;

// 控制状态结构
typedef struct {
    ControlMode_t mode;         // 控制模式
    bool heaterEnabled;         // 加热器使能
    bool fanEnabled;            // 风扇使能
    bool humidifierEnabled;     // 雾化器使能
    uint32_t lastControlTime;   // 最后控制时间
    float tempOutput;           // 温度控制输出
    float humidityOutput;       // 湿度控制输出
} ControlState_t;

// 函数声明
void TempControl_Init(void);
void TempControl_Update(SensorData_t *sensorData, ControlConfig_t *config);
void TempControl_SetMode(ControlMode_t mode);
ControlMode_t TempControl_GetMode(void);
ControlState_t TempControl_GetState(void);

// PID控制函数
void PID_Init(PID_t *pid, float kp, float ki, float kd);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);
void PID_Reset(PID_t *pid);

// 控制算法函数
void TempControl_TemperatureControl(float currentTemp, float targetTemp, float tolerance);
void TempControl_HumidityControl(float currentHumidity, float targetHumidity, float tolerance);
void TempControl_AutoMode(SensorData_t *sensorData, ControlConfig_t *config);
void TempControl_ManualMode(void);

// 配置函数
void TempControl_SetTargetTemperature(float temperature);
void TempControl_SetTargetHumidity(float humidity);
void TempControl_SetTolerance(float tempTolerance, float humidityTolerance);

#ifdef __cplusplus
}
#endif

#endif /* __TEMP_CONTROL_H */ 