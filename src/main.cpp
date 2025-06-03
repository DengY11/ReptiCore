/*
 * STM32F407 温湿度智能控制系统
 * 主程序入口文件 - main.cpp
 * 现代C++ + FreeRTOS + STM32 HAL 重构版本
 */

#include "main.h"

#include <array>
#include <functional>
#include <memory>
#include <string>

#include "DHT22.h"
#include "FreeRTOS.h"
#include "OLED.h"
#include "RelayControl.h"
#include "TempControl.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

// FreeRTOS 任务句柄
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t displayTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;

// 全局变量
extern "C" {
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
}

// 命名空间别名
namespace RC = ReptileController;

// 类型别名（为了兼容C风格接口）
extern "C" {
typedef RC::SensorData SensorData_t;
typedef RC::ControlConfig ControlConfig_t;
}

// 全局实例
RC::SensorData g_sensorData;
RC::ControlConfig g_controlConfig{29.0f, 55.0f, 1.0f, 5.0f};  // 球蟒配置

// 互斥量和信号量
SemaphoreHandle_t sensorDataMutex = nullptr;

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

// 现代C++风格的UART打印函数
void printMessage(const std::string &message) {
  HAL_UART_Transmit(&huart1, reinterpret_cast<const uint8_t *>(message.c_str()),
                    static_cast<uint16_t>(message.length()), 1000);
}

// 使用lambda表达式的传感器数据保护函数
auto protectedSensorDataAccess = [](auto func) -> void {
  if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    func(g_sensorData);
    xSemaphoreGive(sensorDataMutex);
  }
};

int main(void) {
  // HAL 库初始化
  HAL_Init();

  // 配置系统时钟
  SystemClock_Config();

  // 初始化GPIO和外设
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  // 初始化各个模块（现代C++风格）
  RC::DHT22::g_sensor.initialize();
  OLED_Init();
  RC::Relay::g_controller.initialize();
  TempControl_Init();

  // 打印启动信息（使用现代C++字符串）
  printMessage("STM32F407 爬宠温湿度控制系统启动...\r\n");
  printMessage("配置: 球蟒模式 - 目标温度29°C, 目标湿度55%\r\n");

  // 创建互斥量
  sensorDataMutex = xSemaphoreCreateMutex();
  if (sensorDataMutex == nullptr) {
    printMessage("错误: 无法创建互斥量\r\n");
    Error_Handler();
  }

  // 创建 FreeRTOS 任务
  const UBaseType_t sensorPriority = 3;
  const UBaseType_t displayPriority = 2;
  const UBaseType_t controlPriority = 4;
  const uint16_t stackSize = 256;

  if (xTaskCreate(SensorTask, "SensorTask", stackSize, nullptr, sensorPriority,
                  &sensorTaskHandle) != pdPASS) {
    printMessage("错误: 无法创建传感器任务\r\n");
    Error_Handler();
  }

  if (xTaskCreate(DisplayTask, "DisplayTask", stackSize, nullptr,
                  displayPriority, &displayTaskHandle) != pdPASS) {
    printMessage("错误: 无法创建显示任务\r\n");
    Error_Handler();
  }

  if (xTaskCreate(ControlTask, "ControlTask", stackSize, nullptr,
                  controlPriority, &controlTaskHandle) != pdPASS) {
    printMessage("错误: 无法创建控制任务\r\n");
    Error_Handler();
  }

  printMessage("所有任务创建成功，启动调度器...\r\n");

  // 启动调度器
  vTaskStartScheduler();

  // 永远不应该到达这里
  printMessage("错误: 调度器意外退出\r\n");
  while (true) {
    HAL_Delay(1000);
  }
}

// 传感器数据采集任务（现代C++风格）
void SensorTask(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  constexpr TickType_t xFrequency = pdMS_TO_TICKS(2000);  // 2秒周期

  printMessage("[传感器任务] 启动\r\n");

  for (;;) {
    // 读取DHT22传感器数据（使用现代C++结构体）
    const auto sensorResult = RC::DHT22::g_sensor.readData();

    if (sensorResult.valid) {
      const float temperature = sensorResult.temperature;
      const float humidity = sensorResult.humidity;

      // 使用lambda表达式保护共享数据
      protectedSensorDataAccess([&](RC::SensorData &data) {
        data.updateData(temperature, humidity, HAL_GetTick());
      });

      // 发送调试信息（使用格式化字符串）
      const std::string debugMsg =
          "[传感器] 温度: " + std::to_string(temperature) +
          "°C, 湿度: " + std::to_string(humidity) + "%\r\n";
      printMessage(debugMsg);
    } else {
      // 使用lambda表达式标记数据无效
      protectedSensorDataAccess(
          [](RC::SensorData &data) { data.invalidate(); });

      printMessage("[传感器] DHT22读取失败!\r\n");
    }

    // 精确延时
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 显示任务（现代C++风格）
void DisplayTask(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  constexpr TickType_t xFrequency = pdMS_TO_TICKS(500);  // 500ms周期

  printMessage("[显示任务] 启动\r\n");

  for (;;) {
    // 更新OLED显示
    OLED_Clear();

    // 读取传感器数据（使用现代C++拷贝语义）
    RC::SensorData localSensorData{};
    protectedSensorDataAccess([&](const RC::SensorData &data) {
      localSensorData = data;  // 使用默认拷贝构造
    });

    if (localSensorData.isValid()) {
      OLED_ShowTemperature(localSensorData.getTemperature(),
                           g_controlConfig.getTargetTemp());
      OLED_ShowHumidity(localSensorData.getHumidity(),
                        g_controlConfig.getTargetHumidity());
    } else {
      OLED_ShowString(0, 0, "传感器错误", 16);
    }

    OLED_ShowSystemStatus();
    OLED_Refresh();

    // 精确延时
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 控制任务（现代C++风格）
void ControlTask(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  constexpr TickType_t xFrequency = pdMS_TO_TICKS(1000);  // 1秒周期

  printMessage("[控制任务] 启动\r\n");

  for (;;) {
    // 读取传感器数据（使用现代C++拷贝语义）
    RC::SensorData localSensorData{};
    protectedSensorDataAccess(
        [&](const RC::SensorData &data) { localSensorData = data; });

    if (localSensorData.isValid()) {
      // 直接使用现代C++接口进行控制
      if (RC::Control::g_controller) {
        RC::Control::g_controller->update(localSensorData, g_controlConfig);
      }
    }

    // 精确延时
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 系统时钟配置
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

// GPIO初始化（现代C++风格）
static void MX_GPIO_Init(void) {
  // 启用GPIO时钟
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // 配置继电器控制引脚 (PC13, PC14, PC15)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  GPIO_InitTypeDef GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
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
static void MX_USART1_UART_Init(void) {
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
static void MX_I2C1_Init(void) {
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
static void MX_TIM2_Init(void) {
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168 - 1;  // 1MHz计数频率
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_Base_Start(&htim2);
}

// 错误处理函数（现代C++风格）
void Error_Handler(void) {
  __disable_irq();
  printMessage("错误: 系统进入错误处理状态\r\n");

  // 使用constexpr延时
  constexpr uint32_t errorBlinkDelay = 200;

  while (true) {
    // LED闪烁指示错误
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(errorBlinkDelay);
  }
}

// FreeRTOS 静态内存分配回调函数
#if (configSUPPORT_STATIC_ALLOCATION == 1)

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#endif /* configSUPPORT_STATIC_ALLOCATION */