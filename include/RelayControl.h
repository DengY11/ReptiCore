/*
 * 继电器控制模块头文件
 * RelayControl.h
 */

#ifndef __RELAY_CONTROL_H
#define __RELAY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 继电器类型枚举
typedef enum {
    RELAY_HEATER = 0,       // 加热器继电器 (PC13)
    RELAY_FAN,              // 风扇继电器 (PC14)
    RELAY_HUMIDIFIER,       // 雾化器继电器 (PC15)
    RELAY_MAX
} RelayType_t;

// 继电器状态枚举
typedef enum {
    RELAY_OFF = 0,          // 继电器关闭
    RELAY_ON = 1            // 继电器开启
} RelayState_t;

// 继电器控制状态结构
typedef struct {
    RelayState_t heaterState;       // 加热器状态
    RelayState_t fanState;          // 风扇状态
    RelayState_t humidifierState;   // 雾化器状态
    uint32_t lastUpdateTime;        // 最后更新时间
} RelayStatus_t;

// 函数声明
void RelayControl_Init(void);
void RelayControl_Set(RelayType_t relay, RelayState_t state);
RelayState_t RelayControl_Get(RelayType_t relay);
void RelayControl_Toggle(RelayType_t relay);
void RelayControl_AllOff(void);
RelayStatus_t RelayControl_GetStatus(void);

// 安全控制函数
void RelayControl_SafetyCheck(void);
void RelayControl_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_CONTROL_H */ 