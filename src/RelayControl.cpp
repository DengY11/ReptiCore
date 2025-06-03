/*
 * 继电器控制模块实现
 * RelayControl.cpp
 */

#include "RelayControl.h"

// 继电器状态记录
static RelayStatus_t relayStatus = {
    .heaterState = RELAY_OFF,
    .fanState = RELAY_OFF,
    .humidifierState = RELAY_OFF,
    .lastUpdateTime = 0
};

// 继电器引脚映射表
static const struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} relayPins[RELAY_MAX] = {
    {RELAY_HEATER_Port, RELAY_HEATER_Pin},           // 加热器
    {RELAY_FAN_Port, RELAY_FAN_Pin},                 // 风扇
    {RELAY_HUMIDIFIER_Port, RELAY_HUMIDIFIER_Pin}    // 雾化器
};

// 继电器控制初始化
void RelayControl_Init(void)
{
    // 初始化所有继电器为关闭状态
    RelayControl_AllOff();
    
    // 初始化状态结构
    relayStatus.heaterState = RELAY_OFF;
    relayStatus.fanState = RELAY_OFF;
    relayStatus.humidifierState = RELAY_OFF;
    relayStatus.lastUpdateTime = HAL_GetTick();
}

// 设置继电器状态
void RelayControl_Set(RelayType_t relay, RelayState_t state)
{
    if (relay >= RELAY_MAX) return;
    
    // 设置GPIO状态
    GPIO_PinState pinState = (state == RELAY_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(relayPins[relay].port, relayPins[relay].pin, pinState);
    
    // 更新状态记录
    switch (relay) {
        case RELAY_HEATER:
            relayStatus.heaterState = state;
            break;
        case RELAY_FAN:
            relayStatus.fanState = state;
            break;
        case RELAY_HUMIDIFIER:
            relayStatus.humidifierState = state;
            break;
        default:
            break;
    }
    
    relayStatus.lastUpdateTime = HAL_GetTick();
    
    // 执行安全检查
    RelayControl_SafetyCheck();
}

// 获取继电器状态
RelayState_t RelayControl_Get(RelayType_t relay)
{
    if (relay >= RELAY_MAX) return RELAY_OFF;
    
    switch (relay) {
        case RELAY_HEATER:
            return relayStatus.heaterState;
        case RELAY_FAN:
            return relayStatus.fanState;
        case RELAY_HUMIDIFIER:
            return relayStatus.humidifierState;
        default:
            return RELAY_OFF;
    }
}

// 切换继电器状态
void RelayControl_Toggle(RelayType_t relay)
{
    RelayState_t currentState = RelayControl_Get(relay);
    RelayState_t newState = (currentState == RELAY_ON) ? RELAY_OFF : RELAY_ON;
    RelayControl_Set(relay, newState);
}

// 关闭所有继电器
void RelayControl_AllOff(void)
{
    for (int i = 0; i < RELAY_MAX; i++) {
        RelayControl_Set((RelayType_t)i, RELAY_OFF);
    }
}

// 获取继电器状态结构
RelayStatus_t RelayControl_GetStatus(void)
{
    return relayStatus;
}

// 安全检查函数
void RelayControl_SafetyCheck(void)
{
    // 防止加热器和风扇同时工作时间过长（防过热保护）
    static uint32_t lastHeatFanCheck = 0;
    uint32_t currentTime = HAL_GetTick();
    
    if (currentTime - lastHeatFanCheck > 1000) { // 每秒检查一次
        // 如果加热器和风扇都开启超过30分钟，强制关闭加热器5分钟
        static uint32_t heatFanStartTime = 0;
        static bool cooldownActive = false;
        static uint32_t cooldownStartTime = 0;
        
        if (relayStatus.heaterState == RELAY_ON && relayStatus.fanState == RELAY_ON) {
            if (heatFanStartTime == 0) {
                heatFanStartTime = currentTime;
            } else if (currentTime - heatFanStartTime > 30 * 60 * 1000) { // 30分钟
                // 开始冷却周期
                RelayControl_Set(RELAY_HEATER, RELAY_OFF);
                cooldownActive = true;
                cooldownStartTime = currentTime;
                heatFanStartTime = 0;
            }
        } else {
            heatFanStartTime = 0;
        }
        
        // 冷却周期管理
        if (cooldownActive && (currentTime - cooldownStartTime > 5 * 60 * 1000)) { // 5分钟冷却
            cooldownActive = false;
        }
        
        // 如果在冷却周期内，禁止开启加热器
        if (cooldownActive && relayStatus.heaterState == RELAY_ON) {
            RelayControl_Set(RELAY_HEATER, RELAY_OFF);
        }
        
        lastHeatFanCheck = currentTime;
    }
}

// 紧急停止所有继电器
void RelayControl_EmergencyStop(void)
{
    // 立即关闭所有继电器
    for (int i = 0; i < RELAY_MAX; i++) {
        HAL_GPIO_WritePin(relayPins[i].port, relayPins[i].pin, GPIO_PIN_RESET);
    }
    
    // 重置状态
    relayStatus.heaterState = RELAY_OFF;
    relayStatus.fanState = RELAY_OFF;
    relayStatus.humidifierState = RELAY_OFF;
    relayStatus.lastUpdateTime = HAL_GetTick();
} 