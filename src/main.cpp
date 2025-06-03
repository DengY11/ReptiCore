/*
 * STM32F407 温湿度智能控制系统
 * 主程序入口文件 - main.cpp
 * 使用 C++ + FreeRTOS + STM32 HAL
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "DHT22.h"
#include "OLED.h"
#include "RelayControl.h"
#include "TempControl.h"
#include <string.h>

// FreeRTOS 任务句柄
TaskHandle_t sensorTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t controlTaskHandle;

// 全局变量
extern "C" {
    UART_HandleTypeDef huart1;
    I2C_HandleTypeDef hi2c1;
    TIM_HandleTypeDef htim2;
}

// 传感器数据结构
SensorData_t sensorData = {0};

// 控制参数
ControlConfig_t controlConfig = {
    .targetTemp = 25.0f,      // 目标温度 25°C
    .targetHumidity = 60.0f,  // 目标湿度 60%
    .tempTolerance = 1.0f,    // 温度容差 ±1°C
    .humidityTolerance = 5.0f // 湿度容差 ±5%
};

// 互斥量和信号量
SemaphoreHandle_t sensorDataMutex;

// 系统初始化函数声明
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

// FreeRTOS 任务函数声明
extern "C" {
    void SensorTask(void *argument);
    void DisplayTask(void *argument);
    void ControlTask(void *argument);
}

int main(void)
{
    // HAL 库初始化
    HAL_Init();
    
    // 配置系统时钟
    SystemClock_Config();
    
    // 初始化GPIO和外设
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    
    // 初始化各个模块
    DHT22_Init();
    OLED_Init();
    RelayControl_Init();
    TempControl_Init();
    
    // 打印启动信息
    char startMsg[] = "STM32F407 温湿度控制系统启动...\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)startMsg, strlen(startMsg), 1000);
    
    // 创建互斥量
    sensorDataMutex = xSemaphoreCreateMutex();
    
    // 创建 FreeRTOS 任务
    xTaskCreate(SensorTask, "SensorTask", 256, NULL, 3, &sensorTaskHandle);
    xTaskCreate(DisplayTask, "DisplayTask", 256, NULL, 2, &displayTaskHandle);
    xTaskCreate(ControlTask, "ControlTask", 256, NULL, 4, &controlTaskHandle);
    
    // 启动调度器
    vTaskStartScheduler();
    
    // 永远不应该到达这里
    while (1) {
        HAL_Delay(1000);
    }
}

// 传感器数据采集任务
void SensorTask(void *argument)
{
    char debugMsg[100];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2000); // 2秒周期
    
    for(;;)
    {
        // 读取DHT22传感器数据
        float temp, humidity;
        if(DHT22_ReadData(&temp, &humidity) == DHT22_OK)
        {
            // 使用互斥量保护共享数据
            if(xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                sensorData.temperature = temp;
                sensorData.humidity = humidity;
                sensorData.lastUpdateTime = HAL_GetTick();
                sensorData.isValid = true;
                xSemaphoreGive(sensorDataMutex);
            }
            
            // 发送调试信息
            snprintf(debugMsg, sizeof(debugMsg), 
                    "温度: %.1f°C, 湿度: %.1f%%\r\n", 
                    temp, humidity);
            HAL_UART_Transmit(&huart1, (uint8_t*)debugMsg, strlen(debugMsg), 1000);
        }
        else
        {
            // 使用互斥量保护共享数据
            if(xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                sensorData.isValid = false;
                xSemaphoreGive(sensorDataMutex);
            }
            
            HAL_UART_Transmit(&huart1, (uint8_t*)"DHT22读取失败!\r\n", 16, 1000);
        }
        
        // 精确延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 显示任务
void DisplayTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 500ms周期
    
    for(;;)
    {
        // 更新OLED显示
        OLED_Clear();
        
        // 读取传感器数据（使用互斥量保护）
        SensorData_t localSensorData = {0}; // 初始化为0
        if(xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            localSensorData = sensorData;
            xSemaphoreGive(sensorDataMutex);
        }
        
        if(localSensorData.isValid)
        {
            OLED_ShowTemperature(localSensorData.temperature, controlConfig.targetTemp);
            OLED_ShowHumidity(localSensorData.humidity, controlConfig.targetHumidity);
        }
        else
        {
            OLED_ShowString(0, 0, "传感器错误", 16);
        }
        
        OLED_ShowSystemStatus();
        OLED_Refresh();
        
        // 精确延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 控制任务
void ControlTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1秒周期
    
    for(;;)
    {
        // 读取传感器数据（使用互斥量保护）
        SensorData_t localSensorData = {0}; // 初始化为0
        if(xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            localSensorData = sensorData;
            xSemaphoreGive(sensorDataMutex);
        }
        
        if(localSensorData.isValid)
        {
            // 执行温湿度控制逻辑
            TempControl_Update(&localSensorData, &controlConfig);
        }
        
        // 精确延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 系统时钟配置
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

// GPIO初始化
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 配置继电器控制引脚 (PC13, PC14, PC15)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 配置DHT22数据引脚 (PA1)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// USART1初始化
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

// I2C1初始化 (用于OLED)
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

// TIM2初始化 (用于精确延时)
static void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 168-1;  // 1MHz计数频率
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    
    HAL_TIM_Base_Start(&htim2);
}

// 错误处理函数
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        // LED闪烁指示错误
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
}

// FreeRTOS 静态内存分配回调函数（如果启用静态分配）
#if( configSUPPORT_STATIC_ALLOCATION == 1 )

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exist after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is declared static, sizeof( uxIdleTaskStack ) is
    greater than the actual stack size. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exist after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is declared static, sizeof( uxTimerTaskStack ) is
    greater than the actual stack size. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#endif /* configSUPPORT_STATIC_ALLOCATION */ 