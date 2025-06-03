/*
 * 温湿度智能控制算法实现
 * TempControl.cpp
 */

#include "TempControl.h"
#include <math.h>

// 控制状态变量
static ControlState_t controlState = {
    .mode = CONTROL_MODE_AUTO,
    .heaterEnabled = false,
    .fanEnabled = false,
    .humidifierEnabled = false,
    .lastControlTime = 0,
    .tempOutput = 0.0f,
    .humidityOutput = 0.0f
};

// PID控制器
static PID_t tempPID = {0};
static PID_t humidityPID = {0};

// 温控初始化
void TempControl_Init(void)
{
    // 初始化控制状态
    controlState.mode = CONTROL_MODE_AUTO;
    controlState.heaterEnabled = false;
    controlState.fanEnabled = false;
    controlState.humidifierEnabled = false;
    controlState.lastControlTime = HAL_GetTick();
    controlState.tempOutput = 0.0f;
    controlState.humidityOutput = 0.0f;
    
    // 初始化温度PID控制器
    PID_Init(&tempPID, 2.0f, 0.1f, 0.5f);  // Kp=2.0, Ki=0.1, Kd=0.5
    
    // 初始化湿度PID控制器
    PID_Init(&humidityPID, 1.5f, 0.05f, 0.3f);  // Kp=1.5, Ki=0.05, Kd=0.3
}

// 温控主更新函数
void TempControl_Update(SensorData_t *sensorData, ControlConfig_t *config)
{
    if (!sensorData || !config) return;
    
    uint32_t currentTime = HAL_GetTick();
    controlState.lastControlTime = currentTime;
    
    switch (controlState.mode) {
        case CONTROL_MODE_AUTO:
            TempControl_AutoMode(sensorData, config);
            break;
            
        case CONTROL_MODE_MANUAL:
            TempControl_ManualMode();
            break;
            
        case CONTROL_MODE_OFF:
            // 关闭所有设备
            RelayControl_AllOff();
            controlState.heaterEnabled = false;
            controlState.fanEnabled = false;
            controlState.humidifierEnabled = false;
            break;
    }
}

// 自动控制模式
void TempControl_AutoMode(SensorData_t *sensorData, ControlConfig_t *config)
{
    float dt = 1.0f; // 控制周期为1秒
    
    // 温度控制
    float tempError = config->targetTemp - sensorData->temperature;
    controlState.tempOutput = PID_Update(&tempPID, config->targetTemp, sensorData->temperature, dt);
    
    // 湿度控制
    float humidityError = config->targetHumidity - sensorData->humidity;
    controlState.humidityOutput = PID_Update(&humidityPID, config->targetHumidity, sensorData->humidity, dt);
    
    // 温度控制逻辑
    TempControl_TemperatureControl(sensorData->temperature, config->targetTemp, config->tempTolerance);
    
    // 湿度控制逻辑
    TempControl_HumidityControl(sensorData->humidity, config->targetHumidity, config->humidityTolerance);
}

// 手动控制模式
void TempControl_ManualMode(void)
{
    // 手动模式下，继电器状态由外部控制
    // 这里只更新状态记录
    controlState.heaterEnabled = (RelayControl_Get(RELAY_HEATER) == RELAY_ON);
    controlState.fanEnabled = (RelayControl_Get(RELAY_FAN) == RELAY_ON);
    controlState.humidifierEnabled = (RelayControl_Get(RELAY_HUMIDIFIER) == RELAY_ON);
}

// 温度控制算法
void TempControl_TemperatureControl(float currentTemp, float targetTemp, float tolerance)
{
    float tempDiff = targetTemp - currentTemp;
    
    // 温度过低，启动加热器
    if (tempDiff > tolerance) {
        if (!controlState.heaterEnabled) {
            RelayControl_Set(RELAY_HEATER, RELAY_ON);
            controlState.heaterEnabled = true;
        }
        
        // 如果温度差异很大，也启动风扇增强对流
        if (tempDiff > 2.0f * tolerance) {
            if (!controlState.fanEnabled) {
                RelayControl_Set(RELAY_FAN, RELAY_ON);
                controlState.fanEnabled = true;
            }
        }
    }
    // 温度过高，关闭加热器，启动风扇散热
    else if (tempDiff < -tolerance) {
        if (controlState.heaterEnabled) {
            RelayControl_Set(RELAY_HEATER, RELAY_OFF);
            controlState.heaterEnabled = false;
        }
        
        if (!controlState.fanEnabled) {
            RelayControl_Set(RELAY_FAN, RELAY_ON);
            controlState.fanEnabled = true;
        }
    }
    // 温度在正常范围内
    else {
        // 温度接近目标值，关闭加热器
        if (controlState.heaterEnabled && tempDiff < tolerance / 2) {
            RelayControl_Set(RELAY_HEATER, RELAY_OFF);
            controlState.heaterEnabled = false;
        }
        
        // 温度稳定，关闭风扇
        if (controlState.fanEnabled && fabs(tempDiff) < tolerance / 2) {
            RelayControl_Set(RELAY_FAN, RELAY_OFF);
            controlState.fanEnabled = false;
        }
    }
}

// 湿度控制算法
void TempControl_HumidityControl(float currentHumidity, float targetHumidity, float tolerance)
{
    float humidityDiff = targetHumidity - currentHumidity;
    
    // 湿度过低，启动雾化器
    if (humidityDiff > tolerance) {
        if (!controlState.humidifierEnabled) {
            RelayControl_Set(RELAY_HUMIDIFIER, RELAY_ON);
            controlState.humidifierEnabled = true;
        }
    }
    // 湿度过高，关闭雾化器，启动风扇除湿
    else if (humidityDiff < -tolerance) {
        if (controlState.humidifierEnabled) {
            RelayControl_Set(RELAY_HUMIDIFIER, RELAY_OFF);
            controlState.humidifierEnabled = false;
        }
        
        // 湿度过高时启动风扇帮助除湿
        if (humidityDiff < -2.0f * tolerance && !controlState.fanEnabled) {
            RelayControl_Set(RELAY_FAN, RELAY_ON);
            controlState.fanEnabled = true;
        }
    }
    // 湿度在正常范围内
    else {
        // 湿度接近目标值，关闭雾化器
        if (controlState.humidifierEnabled && humidityDiff < tolerance / 2) {
            RelayControl_Set(RELAY_HUMIDIFIER, RELAY_OFF);
            controlState.humidifierEnabled = false;
        }
    }
}

// 设置控制模式
void TempControl_SetMode(ControlMode_t mode)
{
    controlState.mode = mode;
    
    if (mode == CONTROL_MODE_OFF) {
        RelayControl_AllOff();
        PID_Reset(&tempPID);
        PID_Reset(&humidityPID);
    }
}

// 获取控制模式
ControlMode_t TempControl_GetMode(void)
{
    return controlState.mode;
}

// 获取控制状态
ControlState_t TempControl_GetState(void)
{
    return controlState;
}

// PID初始化
void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->lastError = 0.0f;
    pid->maxIntegral = 100.0f;
    pid->outputMax = 100.0f;
    pid->outputMin = -100.0f;
}

// PID更新
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    
    // 比例项
    float proportional = pid->Kp * error;
    
    // 积分项
    pid->integral += error * dt;
    
    // 积分限幅
    if (pid->integral > pid->maxIntegral) {
        pid->integral = pid->maxIntegral;
    } else if (pid->integral < -pid->maxIntegral) {
        pid->integral = -pid->maxIntegral;
    }
    
    float integral = pid->Ki * pid->integral;
    
    // 微分项
    float derivative = pid->Kd * (error - pid->lastError) / dt;
    pid->lastError = error;
    
    // 计算输出
    float output = proportional + integral + derivative;
    
    // 输出限幅
    if (output > pid->outputMax) {
        output = pid->outputMax;
    } else if (output < pid->outputMin) {
        output = pid->outputMin;
    }
    
    return output;
}

// PID重置
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->lastError = 0.0f;
}

// 设置目标温度
void TempControl_SetTargetTemperature(float temperature)
{
    extern ControlConfig_t controlConfig;
    controlConfig.targetTemp = temperature;
    PID_Reset(&tempPID);  // 重置PID以避免积分饱和
}

// 设置目标湿度
void TempControl_SetTargetHumidity(float humidity)
{
    extern ControlConfig_t controlConfig;
    controlConfig.targetHumidity = humidity;
    PID_Reset(&humidityPID);  // 重置PID以避免积分饱和
}

// 设置容差
void TempControl_SetTolerance(float tempTolerance, float humidityTolerance)
{
    extern ControlConfig_t controlConfig;
    controlConfig.tempTolerance = tempTolerance;
    controlConfig.humidityTolerance = humidityTolerance;
} 